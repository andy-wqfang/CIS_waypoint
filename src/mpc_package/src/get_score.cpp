#include <cstdio>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "Eigen/Core"
#include "Eigen/QR"
#include "nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

using std::placeholders::_1;

// for convenience
using json = nlohmann::json;

std::string t_name = "Car_1";
std::string path = "/home/wqfang/research/CIS/src/mpc_package/test_o.json";

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
      RCLCPP_INFO(this->get_logger(), "Score Calculating...!");

      std::ifstream iTrack(path);
      auto j = json::parse(iTrack);
      
      std::vector<double> tmp_x = j["X"];
      std::vector<double> tmp_y = j["Y"];

      this->ptsx = tmp_x;
      this->ptsy = tmp_y;

      sub_ = this ->create_subscription<geometry_msgs::msg::PointStamped>(
        t_name, 50, std::bind(&Mpc_Node::mpc_callback, this, _1));
    }

  private:
    void mpc_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
      auto tic = std::chrono::high_resolution_clock::now();
      
      double px, py, psi;
      
      px = msg->point.x;
      py = msg->point.y;
      psi = msg->point.z;

      int c_index = getCloest(px, py);

      // std::printf("cloest index : %d, %d, %d\n", c_index, ptsx.size(), ptsy.size());

      std::vector<double> r_ptsx = ptsx;
      std::vector<double> r_ptsy = ptsy;

      std::rotate(r_ptsx.begin(), r_ptsx.begin() + c_index, r_ptsx.end());
      std::rotate(r_ptsy.begin(), r_ptsy.begin() + c_index, r_ptsy.end());

      std::vector<double> c_ptsx(r_ptsx.begin(), r_ptsx.begin() + 20);
      std::vector<double> c_ptsy(r_ptsy.begin(), r_ptsy.begin() + 20);
   
      std::vector<double> ptsx_ = c_ptsx;
      std::vector<double> ptsy_ = c_ptsy;

      for (size_t i = 0; i < ptsx_.size(); i++) {

        const double dtx = ptsx_[i] - px;
        const double dty = ptsy_[i] - py;

        ptsx_[i] = dtx * cos(psi) + dty * sin(psi);
        ptsy_[i] = dty * cos(psi) - dtx * sin(psi);

      }

      // Put ptsx and ptsy data into vectors
      Eigen::VectorXd ptsxvec = Eigen::VectorXd::Map(ptsx_.data(), ptsx_.size());
      Eigen::VectorXd ptsyvec = Eigen::VectorXd::Map(ptsy_.data(), ptsy_.size());
      
      auto coeffs = polyfit(ptsxvec, ptsyvec, 3);
      
      const double cte = polyeval(coeffs, 0);
      const double epsi = -atan(coeffs[1]);

      // if(c_index <= 13 || 
      //   (100 <= c_index && c_index <= 146) || 
      //   (251 <= c_index && c_index <= 294) || 
      //   (379 <= c_index && c_index <= 438) || 
      //   537 <= c_index)
      // {
      //   std::cout<< c_index <<std::endl;
      //   v_cte.emplace_back(pow(cte, 2));
      //   v_epsi.emplace_back(pow(psi, 2));
      // }

      v_cte.emplace_back(abs(cte));
      v_epsi.emplace_back(abs(epsi));

      double cte_sum = std::accumulate(v_cte.begin(), v_cte.end(), 0.0);
      double epsi_sum = std::accumulate(v_epsi.begin(), v_epsi.end(), 0.0);

      std::printf(" cte mean  : %f    |   total number = %d\n", cte_sum/v_cte.size(), v_cte.size());
      std::printf(" epsi mean : %f    |   total number = %d\n\n", epsi_sum/v_epsi.size(), v_epsi.size());
    }

    int getCloest(double p_x, double p_y)
    {
      int n_points = ptsx.size();
      double min_dist = 1000.0;
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

    std::vector<double> ptsx;
    std::vector<double> ptsy;

    std::vector<double> result;

    Eigen::VectorXd prev_pos;

    const double Lf = 0.17;
    const double dt = 0.1;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
};


int main(int argc, char* argv[])
{  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mpc_Node>();
  
  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }

  return 0;  

}
