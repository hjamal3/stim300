#include "driver_stim300.h"
#include "serial_unix.h"
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

constexpr int defaultSampleRate{ 125 };
constexpr double averageAllanVarianceOfGyro{ 0.0001 * 2 * 0.00046 };
constexpr double averageAllanVarianceOfAcc{ 100 * 2 * 0.0052 };

Eigen::Quaterniond estimate_orientation(const Eigen::Vector3d& acc)
{
  double yaw = atan2(-acc[1], acc[0]);
  double pitch = atan2(acc[2], sqrt(acc[0] * acc[0] + acc[1] * acc[1]));

  Eigen::Quaterniond quat = Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return quat;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stim300_driver_node");

  ros::NodeHandle node;
  std::string imu_path;
  std::string imu_link;
  std::string imu_output;
  node.param<std::string>("device_path", imu_path, "/dev/ttyUSB0");
  node.param<std::string>("imu_frame", imu_link, "imu_link");
  node.param<std::string>("imu_output", imu_output, "/stim300_imu_fb");

  std::string device_name;
  double stanardDeivationOfGyro{ 0 };
  double stanardDeviationOfAcc{ 0 };
  double varianceOfGyro{ 0 };
  double varianceOfAcc{ 0 };
  int sampleRate{ 0 };

  SerialUnix serial_driver(imu_path);
  DriverStim300 driver_stim300(serial_driver);

  node.param("stanard_deviation_of_gyro", stanardDeivationOfGyro, averageAllanVarianceOfGyro);
  node.param("stanard_deviation_of_acc", stanardDeviationOfAcc, averageAllanVarianceOfAcc);
  node.param("sample_rate", sampleRate, defaultSampleRate);
  varianceOfGyro = sampleRate * pow(stanardDeivationOfGyro, 2);
  varianceOfAcc = sampleRate * pow(stanardDeviationOfAcc, 2);

  sensor_msgs::Imu imu_msg_template{};
  imu_msg_template.orientation_covariance[0] = 0.05;
  imu_msg_template.orientation_covariance[4] = 0.01;
  imu_msg_template.orientation_covariance[8] = 0.05;
  imu_msg_template.angular_velocity_covariance[0] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[4] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[8] = varianceOfGyro;
  imu_msg_template.linear_acceleration_covariance[0] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[4] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[8] = varianceOfAcc;
  imu_msg_template.orientation.x = 0;
  imu_msg_template.orientation.y = 0;
  imu_msg_template.orientation.z = 0;
  imu_msg_template.header.frame_id = imu_link;

  ros::Publisher imuSensorPublisher = node.advertise<sensor_msgs::Imu>(imu_output, 1000);

  ros::Rate loop_rate(125);

  ROS_INFO("STIM300 IMU initialized successfully");

  while (ros::ok())
  {
    sensor_msgs::Imu stim300msg = imu_msg_template;

    stim300msg.header.stamp = ros::Time::now();

    if (driver_stim300.processPacket())
    {
      if (!driver_stim300.isChecksumGood())
      {
        ROS_WARN("stim300 CRC error ");
        continue;
      }

      if (!driver_stim300.isSensorStatusGood())
      {
        ROS_WARN("STIM300: Internal hardware error");
        continue;
      }
      Eigen::Vector3d linear_acceleration = driver_stim300.getAccData();
      Eigen::Vector3d gyro_velocities = driver_stim300.getGyroData();
      Eigen::Vector3d incl_acceleration = driver_stim300.getInclData();

      Eigen::Quaterniond orientation = estimate_orientation(incl_acceleration);

      stim300msg.linear_acceleration.x = linear_acceleration[0];
      stim300msg.linear_acceleration.y = linear_acceleration[1];
      stim300msg.linear_acceleration.z = linear_acceleration[2];

      stim300msg.angular_velocity.x = gyro_velocities[0];
      stim300msg.angular_velocity.y = gyro_velocities[1];
      stim300msg.angular_velocity.z = gyro_velocities[2];

      stim300msg.orientation.x = orientation.x();
      stim300msg.orientation.y = orientation.y();
      stim300msg.orientation.z = orientation.z();
      stim300msg.orientation.w = orientation.w();

      // stim300msg.linear_acceleration.x = driver_stim300.getAccX() + 0.0023;
      // stim300msg.linear_acceleration.y = driver_stim300.getAccY() + 0.05;
      // stim300msg.linear_acceleration.z = driver_stim300.getAccZ() + 0.027;
      // stim300msg.angular_velocity.x = driver_stim300.getGyroX();
      // stim300msg.angular_velocity.y = driver_stim300.getGyroY();
      // stim300msg.angular_velocity.z = driver_stim300.getGyroZ();
      imuSensorPublisher.publish(stim300msg);
    }

    loop_rate.sleep();

    ros::spinOnce();
  }
  return 0;
}
