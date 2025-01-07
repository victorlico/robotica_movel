#include <functional>

#include <robotica_movel/robotica_movel.hpp>


namespace robotica_movel {


/**
 * @brief Constructor
 *
 * @param options node options
 */
RoboticaMovel::RoboticaMovel() : Node("robotica_movel") {

  this->setup();
}


/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void RoboticaMovel::setup() {

  // publisher for publishing outgoing messages
  publisher_ = this->create_publisher<std_msgs::msg::Int32>("~/output", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RoboticaMovel::timer_callback, this));
}


/**
 * @brief Timer callback
 */
void RoboticaMovel::timer_callback() {
  auto message = std_msgs::msg::Int32();
  message.data = 42;
  publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);

}

}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotica_movel::RoboticaMovel>());
  rclcpp::shutdown();

  return 0;
}
