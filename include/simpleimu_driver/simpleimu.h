#ifndef SIMPLEIMU_DRIVER
#define SIMPLEIMU_DRIVER

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class Simpleimu
{
public:
  Simpleimu(std::string port, uint32_t baud);

  ~Simpleimu();

private:
  void uartCallback(const boost::system::error_code& error, std::size_t bytes_transferred);

  void listenUart();

  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  sensor_msgs::Imu imu_msg_;

  uint8_t receive_buffer_[42];
  std::vector<uint8_t> msg_buffer_;

  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  boost::thread io_thread_;
};

#endif
