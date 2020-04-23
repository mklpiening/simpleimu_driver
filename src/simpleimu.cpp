#include "simpleimu_driver/simpleimu.h"

Simpleimu::Simpleimu() : io_(), serial_(io_)
{
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_data", 10);

  std::string port;
  int baudrate;

  ros::NodeHandle nh_p("~");
  nh_p.param("port", port, std::string("/dev/ttyUSB0"));
  nh_p.param("baudrate", baudrate, 115200);

  ROS_INFO_STREAM("connecting to device ...");

  boost::system::error_code ec;
  serial_.open(port, ec);
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));

  sleep(2);

  listenUart();

  boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));
  io_thread_.swap(t);

  ROS_INFO_STREAM("connected!");
}

Simpleimu::~Simpleimu()
{
  io_.stop();
}

void Simpleimu::uartCallback(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (!error)  // && bytes_transferred == 42)
  {
    msg_buffer_.insert(msg_buffer_.end(), &receive_buffer_[0], &receive_buffer_[bytes_transferred]);

    while (msg_buffer_.size() >= 42 && (msg_buffer_[0] != 0x80 || msg_buffer_[41] != 0x40))
    {
      msg_buffer_.erase(msg_buffer_.begin());
    }

    if (msg_buffer_.size() >= 42)
    {
      // linear accelerations
      int lin_x_raw = 0;
      lin_x_raw = lin_x_raw | (msg_buffer_[1]) | (((int)msg_buffer_[2]) << 8) | (((int)msg_buffer_[3]) << 16) |
                  (((int)msg_buffer_[4]) << 24);

      int lin_y_raw = 0;
      lin_y_raw = lin_y_raw | (msg_buffer_[5]) | (((int)msg_buffer_[6]) << 8) | (((int)msg_buffer_[7]) << 16) |
                  (((int)msg_buffer_[8]) << 24);

      int lin_z_raw = 0;
      lin_z_raw = lin_z_raw | (msg_buffer_[9]) | (((int)msg_buffer_[10]) << 8) | (((int)msg_buffer_[11]) << 16) |
                  (((int)msg_buffer_[12]) << 24);

      // angular accelerations
      int rot_x_raw = 0;
      rot_x_raw = rot_x_raw | (msg_buffer_[13]) | (((int)msg_buffer_[14]) << 8) | (((int)msg_buffer_[15]) << 16) |
                  (((int)msg_buffer_[16]) << 24);

      int rot_y_raw = 0;
      rot_y_raw = rot_y_raw | (msg_buffer_[17]) | (((int)msg_buffer_[18]) << 8) | (((int)msg_buffer_[19]) << 16) |
                  (((int)msg_buffer_[20]) << 24);

      int rot_z_raw = 0;
      rot_z_raw = rot_z_raw | (msg_buffer_[21]) | (((int)msg_buffer_[22]) << 8) | (((int)msg_buffer_[23]) << 16) |
                  (((int)msg_buffer_[24]) << 24);

      // orientaion
      int orientation_x_raw = 0;
      orientation_x_raw = orientation_x_raw | (msg_buffer_[25]) | (((int)msg_buffer_[26]) << 8) |
                          (((int)msg_buffer_[27]) << 16) | (((int)msg_buffer_[28]) << 24);

      int orientation_y_raw = 0;
      orientation_y_raw = orientation_y_raw | (msg_buffer_[29]) | (((int)msg_buffer_[30]) << 8) |
                          (((int)msg_buffer_[31]) << 16) | (((int)msg_buffer_[32]) << 24);

      int orientation_z_raw = 0;
      orientation_z_raw = orientation_z_raw | (msg_buffer_[33]) | (((int)msg_buffer_[34]) << 8) |
                          (((int)msg_buffer_[35]) << 16) | (((int)msg_buffer_[36]) << 24);

      int orientation_w_raw = 0;
      orientation_w_raw = orientation_w_raw | (msg_buffer_[37]) | (((int)msg_buffer_[38]) << 8) |
                          (((int)msg_buffer_[39]) << 16) | (((int)msg_buffer_[40]) << 24);

      for (int i = 0; i < 42; i++)
      {
        msg_buffer_.erase(msg_buffer_.begin());
      }

      imu_msg_.header.stamp = ros::Time::now();
      imu_msg_.header.frame_id = "imu";

      imu_msg_.linear_acceleration.x = (float)lin_x_raw / 1000.0;
      imu_msg_.linear_acceleration.y = (float)lin_y_raw / 1000.0;
      imu_msg_.linear_acceleration.z = (float)lin_z_raw / 1000.0;

      imu_msg_.angular_velocity.x = (float)rot_x_raw / 1000.0;
      imu_msg_.angular_velocity.y = (float)rot_y_raw / 1000.0;
      imu_msg_.angular_velocity.z = (float)rot_z_raw / 1000.0;

      imu_msg_.orientation.x = (float)orientation_x_raw / 1000.0;
      imu_msg_.orientation.y = (float)orientation_y_raw / 1000.0;
      imu_msg_.orientation.z = (float)orientation_z_raw / 1000.0;
      imu_msg_.orientation.w = (float)orientation_w_raw / 1000.0;

      imu_pub_.publish(imu_msg_);
    }
  }

  listenUart();
}

void Simpleimu::listenUart()
{
  serial_.async_read_some(boost::asio::buffer(receive_buffer_, 42),
                          boost::bind(&Simpleimu::uartCallback, this, boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));
}
