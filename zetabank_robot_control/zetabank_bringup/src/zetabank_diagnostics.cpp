/* Authors: Geonhee LEE */

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <zetabank_msgs/SensorState.h>
#include <zetabank_msgs/VersionInfo.h>

#define SOFTWARE_VERSION "2.0.0"
#define FIRMWARE_VERSION "2.0.0"
#define HARDWARE_VERSION "2.0.0"

ros::Publisher zb_diagnostics_pub;
/*
(http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html)
diagnostic_msgs/DiagnosticStatus.msg

# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status

*/
diagnostic_msgs::DiagnosticArray zb_diagnostics;

diagnostic_msgs::DiagnosticStatus imu_state;
diagnostic_msgs::DiagnosticStatus motor_state;
diagnostic_msgs::DiagnosticStatus Lidar_state;
diagnostic_msgs::DiagnosticStatus battery_state;
diagnostic_msgs::DiagnosticStatus button_state;

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

void setIMUDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "MPU9250");
}

void setMotorDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&motor_state, level, "Actuator", message, "BLDC_Motor");
}

void setBatteryDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&battery_state, level, "Power System", message, "Battery");
}

void setLDSDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&Lidar_state, level, "Lidar Sensor", message, "TIM561");
}

void setButtonDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&button_state, level, "Analog Button", message, "OpenCR Button");
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void LidarMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void sensorStateMsgCallback(const zetabank_msgs::SensorState::ConstPtr &msg)
{
  if (msg->battery > 24.0)
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  else
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Charge!!! Charge!!!");

  if (msg->button == zetabank_msgs::SensorState::BUTTON0)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 0 IS PUSHED");
  else if (msg->button == zetabank_msgs::SensorState::BUTTON1)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 1 IS PUSHED");
  else
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Pushed Nothing");

  if (msg->left_encoder > 0 )
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Left Motor Torque ON");
  else
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Left Motor Torque OFF");  
  if (msg->right_encoder > 0 )
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Right Motor Torque ON");
  else
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Right Motor Torque OFF");

}


void msgPub()
{
  zb_diagnostics.header.stamp = ros::Time::now();

  zb_diagnostics.status.clear();
  zb_diagnostics.status.push_back(imu_state);
  zb_diagnostics.status.push_back(motor_state);
  zb_diagnostics.status.push_back(Lidar_state);
  zb_diagnostics.status.push_back(battery_state);
  zb_diagnostics.status.push_back(button_state);

  zb_diagnostics_pub.publish(zb_diagnostics);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zetabank_diagnostic");
  ros::NodeHandle nh;

  zb_diagnostics_pub  = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);


  ros::Subscriber imu         = nh.subscribe("/imu", 10, imuMsgCallback);
  ros::Subscriber lds         = nh.subscribe("/scan", 10, LidarMsgCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
