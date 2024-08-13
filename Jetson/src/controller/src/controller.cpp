#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "controller/define.h"
#include "controller/spi.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0),spi_("/dev/spidev0.0")
    {
      spi_.set_max_speed_hz(1000000);
      spi_.set_bits_per_word(8);
      spi_.set_mode(0);
      spi_.set_lsb(0);

      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      cmd_.vel_l = 1.0f;
      cmd_.vel_r = 1.0f;

      static auto spi_data = [&]() {
        kiks::io::spi::Data spi_data = {};
        spi_data.len = sizeof(cmd_);
        spi_data.speed_hz = 1000000;
        spi_data.bits_per_word = 8;
        spi_data.tx_buf = reinterpret_cast<std::uint64_t>(&cmd_);
        spi_data.rx_buf = reinterpret_cast<std::uint64_t>(&res_);
        return spi_data;
      }();
      spi_.write(&spi_data);

      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    
    kiks::io::spi::Master spi_;
    Command cmd_;
    Result res_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}