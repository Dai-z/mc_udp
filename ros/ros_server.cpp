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

std::unique_ptr<mc_udp::Server> Server;
double Reading[6];

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "sensor_server");
  ros::NodeHandle nh("~");

  int port = 4444;
  nh.getParam("port", port);
  ROS_INFO("Start sensor server at port: %d", port);
  Server = std::make_unique<mc_udp::Server>(port);
  initSensors();

  auto imu_sub = nh.subscribe("/real/imu", 1, imuCallback);
  auto joint_sub = nh.subscribe("/real/joint_states", 1, jointCallback);
  auto lft_sub = nh.subscribe("/real/force/LeftFootForceSensor", 1, leftFootCallback);
  auto rft_sub = nh.subscribe("/real/force/RightFootForceSensor", 1, rightFootCallback);

  // TODO: check main loop logic.
  // TODO: check when to send sensor data. Wait until collected? How to deal with missing data?
  auto & sensors = Server->sensors().messages["dummy"];
  size_t control_id = 0;
  while(ros::ok())
  {
    ros::spinOnce();
    auto start_t = std::chrono::system_clock::now();
    Server->send();
    auto send_t = std::chrono::system_clock::now();
    if(Server->recv())
    {
      const auto & control = Server->control().messages.at("dummy");
      control_id = control.id;
      if(control.id != sensors.id - 1)
      {
        ROS_WARN("[ros] Server control id %d does not match sensors id %d\n", control_id, sensors.id);
      }
    }
    auto recv_t = std::chrono::system_clock::now();
    std::chrono::duration<double> send_dt = send_t - start_t;
    std::chrono::duration<double> recv_dt = recv_t - send_t;
    std::cout << "send_dt: " << send_dt.count() * 1000 << ", recv_dt: " << recv_dt.count() * 1000
              << ", control_id: " << control_id << "\n";
    sensors.id += 1;
    // 2ms
    usleep(2000);
  }

  return 0;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr & msg)
{
  ROS_DEBUG("IMU received");
  auto & sensors = Server->sensors().messages["dummy"];
  // TODO: check [0,1,2] is [o.x, o.y, o.z]
  sensors.orientation[0] = msg->orientation.x;
  sensors.orientation[1] = msg->orientation.y;
  sensors.orientation[2] = msg->orientation.z;
  // TODO: check [0,1,2] is [a.x, a.y, a.z]
  sensors.angularVelocity[0] = msg->angular_velocity.x;
  sensors.angularVelocity[1] = msg->angular_velocity.y;
  sensors.angularVelocity[2] = msg->angular_velocity.z;
  // TODO: check same concern as above
  sensors.linearAcceleration[0] = msg->linear_acceleration.x;
  sensors.linearAcceleration[1] = msg->linear_acceleration.y;
  sensors.linearAcceleration[2] = msg->linear_acceleration.z;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr & msg)
{
  ROS_DEBUG("joint received");
  auto & sensors = Server->sensors().messages["dummy"];
  // TODO: where to put joint posotions? need extra definition in RobotSensors?
}

void leftFootCallback(const geometry_msgs::WrenchStamped::ConstPtr & msg)
{
  ROS_DEBUG("left foot received");
  auto & sensors = Server->sensors().messages["dummy"];
  // TODO: check order of data in fsensor
  Reading[0] = msg->wrench.force.x;
  Reading[1] = msg->wrench.force.y;
  Reading[2] = msg->wrench.force.z;
  Reading[3] = msg->wrench.torque.x;
  Reading[4] = msg->wrench.torque.y;
  Reading[5] = msg->wrench.torque.z;
  sensors.fsensor("lfsensor", Reading);
}

void rightFootCallback(const geometry_msgs::WrenchStamped::ConstPtr & msg)
{
  ROS_DEBUG("right foot received");
  auto & sensors = Server->sensors().messages["dummy"];
  Reading[0] = msg->wrench.force.x;
  Reading[1] = msg->wrench.force.y;
  Reading[2] = msg->wrench.force.z;
  Reading[3] = msg->wrench.torque.x;
  Reading[4] = msg->wrench.torque.y;
  Reading[5] = msg->wrench.torque.z;
  sensors.fsensor("rfsensor", Reading);
}

void initSensors()
{
  auto & sensors = Server->sensors().messages["ros"];
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