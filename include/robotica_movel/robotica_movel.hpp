#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace robotica_movel {



class RoboticaMovel : public rclcpp::Node {

 public:

  RoboticaMovel();

 private:

  void setup();
  void timer_callback();

 private:

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


}
