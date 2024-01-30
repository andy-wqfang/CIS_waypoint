#include <cstdio>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "MPC.h"
#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;

// for convenience
using json = nlohmann::json;
Eigen::VectorXd prev_pos(3);

std::string t_name = "Car_1";
std::string path = "/home/wqfang/research/CIS/src/mpc_package/lane_16.json";

std::vector<double> v_cte;
std::vector<double> v_epsi;

json jsonTrack;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


Eigen::VectorXd polyfit(Eigen::VectorXd xvals, const Eigen::VectorXd &yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


class Mpc_Node : public rclcpp::Node
{
  public:
    Mpc_Node() : Node("mpc_node")
    {
      RCLCPP_INFO(this->get_logger(), "MPC running...!");

      std::ifstream iTrack(path);
      auto j = json::parse(iTrack);
      
      auto mpc = std::make_shared<MPC>();
      mpc_ = mpc;
      
      std::vector<double> tmp_x = j["X"];
      std::vector<double> tmp_y = j["Y"];

      this->ptsx = tmp_x;
      this->ptsy = tmp_y;

      sub_ = this ->create_subscription<geometry_msgs::msg::PointStamped>(
        t_name, 50, std::bind(&Mpc_Node::mpc_callback, this, _1));
      pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        t_name+"_tmp", 10);

      pub_waypoint = this->create_publisher<nav_msgs::msg::Path>(
        "Waypoint_Path", 100); 
      pub_poly = this->create_publisher<nav_msgs::msg::Path>(
        "Poly_Path", 100);     
      pub_gth = this->create_publisher<nav_msgs::msg::Path>(
        "Ground_Path", 100);  

      // std::this_thread::sleep_for(chrono::milliseconds(5000));
    }

  private:
    void mpc_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
      auto tic = std::chrono::high_resolution_clock::now();
      
      double px, py, psi, v, a, delta;
      
      px = msg->point.x;
      py = msg->point.y;
      psi = msg->point.z;

      rclcpp::Time topic_time = rclcpp::Clock(RCL_ROS_TIME).now();

      p_gth.header.frame_id = "map";

      if(pub_gt)
      {
        geometry_msgs::msg::PoseStamped t_gth;

        p_gth.poses.clear();

        for(int i = 0; i < ptsx.size(); i ++)
        {
          t_gth.pose.position.x = ptsx[i]; // + px;
          t_gth.pose.position.y = ptsy[i]; // + py;
          t_gth.header.frame_id = p_gth.header.frame_id;
          t_gth.header.stamp = topic_time;

          p_gth.poses.emplace_back(t_gth);
        }

        pub_gt = false;
      }

      int c_index = getCloest(px, py);
      std::rotate(ptsx.begin(), ptsx.begin() + c_index, ptsx.end());
      std::rotate(ptsy.begin(), ptsy.begin() + c_index, ptsy.end());

      // std::vector<double> c_ptsx(ptsx.end() - 10, ptsx.end());
      // c_ptsx.insert(c_ptsx.end(), ptsx.begin(), ptsx.begin() + 15);

      // std::vector<double> c_ptsy(ptsy.end() - 10, ptsy.end());
      // c_ptsy.insert(c_ptsx.end(), ptsx.begin(), ptsx.begin() + 15);

      std::vector<double> c_ptsx(ptsx.begin(), ptsx.begin() + 80);
      std::vector<double> c_ptsy(ptsy.begin(), ptsy.begin() + 80);

      Eigen::VectorXd cur_pos(2);
      cur_pos << px, py;

      if(is_init)
      {
        prev_pos = cur_pos;
        v = 0.0;
        a = 0.0;
        delta = 0.0;

        is_init = false;
      }

      else
      {
        v = getVelocity(cur_pos, prev_pos);
        prev_pos = cur_pos;
        a = result[1];
        delta = result[0]; // /(deg2rad(25)*Lf);
      }

      RCLCPP_INFO(this->get_logger(), "velocity : %f", v);
   
      std::vector<double> ptsx_ = c_ptsx;
      std::vector<double> ptsy_ = c_ptsy;

      auto tic_0 = std::chrono::high_resolution_clock::now();

      for (size_t i = 0; i < ptsx_.size(); i++) {

        const double dtx = ptsx_[i] - px;
        const double dty = ptsy_[i] - py;

        ptsx_[i] = dtx * cos(psi) + dty * sin(psi);
        ptsy_[i] = dty * cos(psi) - dtx * sin(psi);

      }

      auto tok_0 = std::chrono::high_resolution_clock::now();

      // Put ptsx and ptsy data into vectors
      Eigen::VectorXd ptsxvec = Eigen::VectorXd::Map(ptsx_.data(), ptsx_.size());
      Eigen::VectorXd ptsyvec = Eigen::VectorXd::Map(ptsy_.data(), ptsy_.size());
      
      auto coeffs = polyfit(ptsxvec, ptsyvec, 3);
      
      // Plot Poly and Waypoint ->

      geometry_msgs::msg::PoseStamped tmp_w;
      geometry_msgs::msg::PoseStamped tmp_p;

      t_poly.poses.clear();
      t_waypoint.poses.clear();

      t_poly.header.frame_id = "map";
      t_waypoint.header.frame_id = "map";

      for(int i = 0; i < ptsxvec.size(); i++)
      {
        double tmp_x = ptsxvec(i);
        double tmp_y = polyeval(coeffs, ptsxvec(i));

        tmp_p.pose.position.x  = tmp_x * cos(psi) - tmp_y * sin(psi) + px;
        tmp_p.pose.position.y  = tmp_y * cos(psi) + tmp_x * sin(psi) + py;       
        
        // tmp_p.pose.position.x = ptsxvec(i) + px;
        // tmp_p.pose.position.y = polyeval(coeffs, ptsxvec(i)) + py;
        tmp_p.header.frame_id = t_poly.header.frame_id ;
        tmp_p.header.stamp = topic_time;

        t_poly.poses.emplace_back(tmp_p);

        tmp_w.pose.position.x = c_ptsx[i]; // + px;
        tmp_w.pose.position.y = c_ptsy[i]; // + py;
        tmp_w.header.frame_id = t_waypoint.header.frame_id ;
        tmp_w.header.stamp = topic_time;

        t_waypoint.poses.emplace_back(tmp_w);
      }

      pub_waypoint->publish(t_waypoint);
      pub_poly->publish(t_poly);
      pub_gth->publish(p_gth);

      // <- End Plot!

      const double cte = polyeval(coeffs, 0);
      const double epsi = -atan(coeffs[1]);

      RCLCPP_INFO(this->get_logger(), "cte : %f  epsi : %f\n", cte, epsi);

      Eigen::VectorXd state(6);

      // Predict (x = y = psi = 0)

      double predicted_x = v * dt;
      double predicted_y = 0;
      double predicted_psi = - v * delta / Lf * dt;
      double predicted_v = v + a * dt;
      double predicted_cte = cte + v * CppAD::sin(epsi) * dt;
      double predicted_epsi = epsi + predicted_psi;
      
      // double predicted_x = 0;
      // double predicted_y = 0;
      // double predicted_psi = -delta;
      // double predicted_v = v;
      // double predicted_cte = cte;
      // double predicted_epsi = epsi;
      
      state << predicted_x, predicted_y, predicted_psi, predicted_v, predicted_cte, predicted_epsi;
      
      auto tic_1 = std::chrono::high_resolution_clock::now();
      // Solve using MPC
      // coeffs to predict future cte and epsi
      result = mpc_->Solve(state, coeffs);
      auto tok_1 = std::chrono::high_resolution_clock::now();

      double r_steer = result[0]; // / (deg2rad(25) * Lf);

      if (r_steer > M_PI)
        r_steer -= 2.0 * M_PI;
      if (r_steer < -M_PI)
        r_steer += 2.0 * M_PI;

      t_pub.point.x = result[1];
      t_pub.point.y = rad2deg(r_steer);
      t_pub.point.z = 0;

      pub_->publish(t_pub);

      auto tok = std::chrono::high_resolution_clock::now();
      // std::chrono::duration<double> time_span_0 = std::chrono::duration_cast<std::chrono::duration<double>>(tok_0-tic_0);
      std::chrono::duration<double> time_span_1 = std::chrono::duration_cast<std::chrono::duration<double>>(tok_1-tic_1);
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(tok-tic);
      
      RCLCPP_INFO(this->get_logger(), "Full Code : %f MPC Calc = %f", time_span, time_span_1);
      RCLCPP_INFO(this->get_logger(), "accel : %f  delta : %f\n", t_pub.point.x, t_pub.point.y);
    }

    double getVelocity(Eigen::VectorXd cur_pos, Eigen::VectorXd prev_pos)
    {
      Eigen::VectorXd diff_pos = cur_pos - prev_pos;

      double diff_dis = diff_pos.transpose() * diff_pos;

      double vel = sqrt(diff_dis) / dt;

      return vel;
    }

    int getCloest(double p_x, double p_y)
    {
      int n_points = ptsx.size();
      double min_dist = 100.0;
      int min_idx =0;
      
      for(int i=0; i<n_points-1; i++)
      {
          double dx = ptsx[i]- p_x;
          double dy = ptsy[i]- p_y;
          double dist = std::sqrt(dx*dx + dy*dy);    //dist is straight line distance between points
          if (dist < min_dist) { 
              min_dist = dist;
              min_idx = i;
          }
      }

      return min_idx;
    }

    bool is_init = true;

    bool pub_gt = true;

    std::vector<double> ptsx;
    std::vector<double> ptsy;

    std::vector<double> result;

    Eigen::VectorXd prev_pos;

    const double Lf = 0.17;
    const double dt = 0.1;

    std::shared_ptr<MPC> mpc_;

    geometry_msgs::msg::PointStamped t_pub;

    nav_msgs::msg::Path t_waypoint;
    nav_msgs::msg::Path t_poly;
  
    nav_msgs::msg::Path p_gth;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_waypoint;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_poly;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_gth;
};


int main(int argc, char* argv[])
{
  // //Display the MPC predicted trajectory
  // vector<double> mpc_x_vals;
  // vector<double> mpc_y_vals;

  // //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
  // // the points in the simulator are connected by a Green line
  // // std::cout << "result size: " << result.size() << endl;
  // for (size_t i = 2; i < result.size(); i++) {
  //   if(i%2 == 0){
  //     mpc_x_vals.push_back(result[i]);
  //   } else {
  //     mpc_y_vals.push_back(result[i]);
  //   }
  // }

  // //Display the waypoints/reference line
  // vector<double> next_x_vals;
  // vector<double> next_y_vals;

  // next_x_vals.resize(static_cast<unsigned long>(ptsxvec.size()));
  // next_y_vals.resize(static_cast<unsigned long>(ptsyvec.size()));


  // //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
  // // the points in the simulator are connected by a Yellow line
  // for (int i = 0; i < ptsxvec.size(); i++) {
  //   next_x_vals[i] = ptsxvec[i];
  //   next_y_vals[i] = ptsyvec[i];
  // }
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mpc_Node>();
  
  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }

  return 0;  

}
