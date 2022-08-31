#include <mc_udp/server/Server.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <unistd.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr & msg);
void jointCallback(const sensor_msgs::JointState::ConstPtr & msg);
void leftFootCallback(const geometry_msgs::WrenchStamped::ConstPtr & msg);
void rightFootCallback(const geometry_msgs::WrenchStamped::ConstPtr & msg);
void initSensors();

std::unique_ptr<mc_udp::Server> server;

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "sensor_server");
  ros::NodeHandle nh("~");

  int port = 4444;
  nh.getParam("port", port);
  ROS_INFO("Start sensor server at port: %d", port);
  server = std::make_unique<mc_udp::Server>(port);
  initSensors();

  auto imu_sub = nh.subscribe("/real/imu", 10, imuCallback);
  auto joint_sub = nh.subscribe("/real/joint_states", 10, jointCallback);
  auto lft_sub = nh.subscribe("/real/force/LeftFootForceSensor", 10, leftFootCallback);
  auto rft_sub = nh.subscribe("/real/force/RightFootForceSensor", 10, rightFootCallback);
  ros::spin();

  size_t control_id = 0;
  auto & sensors = server->sensors().messages["ros"];
  while(1)
  {
    auto start_t = std::chrono::system_clock::now();
    server->send();
    auto send_t = std::chrono::system_clock::now();
    if(server->recv())
    {
      const auto & control = server->control().messages.at("ros");
      control_id = control.id;
      if(control.id != sensors.id - 1)
      {
        std::cout << "[ros] Server control id " << control.id << " does not match sensors id " << sensors.id
                  << std::endl;
      }
    }
    auto recv_t = std::chrono::system_clock::now();
    std::chrono::duration<double> send_dt = send_t - start_t;
    std::chrono::duration<double> recv_dt = recv_t - send_t;
    std::cout << "send_dt: " << send_dt.count() * 1000 << ", recv_dt: " << recv_dt.count() * 1000
              << ", control_id: " << control_id << "\n";
    sensors.id += 1;
    usleep(200000);
  }
  return 0;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr & msg)
{
  ROS_INFO("IMU received");
}
void jointCallback(const sensor_msgs::JointState::ConstPtr & msg)
{
  ROS_INFO("joint received");
}
void leftFootCallback(const geometry_msgs::WrenchStamped::ConstPtr & msg)
{
  ROS_INFO("left foot received");
}
void rightFootCallback(const geometry_msgs::WrenchStamped::ConstPtr & msg)
{
  ROS_INFO("right foot received");
}

void initSensors()
{
  auto & sensors = server->sensors().messages["ros"];
  sensors.encoders = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  sensors.torques = {100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110};
  double reading[6];
  for(size_t i = 0; i < 6; ++i)
  {
    reading[i] = i + 1;
  }
  sensors.fsensor("rfsensor", reading);
  sensors.fsensor("lfsensor", reading);
  sensors.fsensor("rhsensor", reading);
  sensors.fsensor("lhsensor", reading);
  sensors.orientation[0] = 1;
  sensors.orientation[1] = 2;
  sensors.orientation[2] = 3;
  sensors.angularVelocity[0] = 4;
  sensors.angularVelocity[1] = 5;
  sensors.angularVelocity[2] = 6;
  sensors.linearAcceleration[0] = 7;
  sensors.linearAcceleration[1] = 8;
  sensors.linearAcceleration[2] = 9;
  sensors.id = 0;
}