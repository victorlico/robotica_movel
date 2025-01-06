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
}


}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotica_movel::RoboticaMovel>());
  rclcpp::shutdown();

  return 0;
}
