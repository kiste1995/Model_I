#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include <serial/serial.h>

#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>


#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <boost/thread/thread.hpp>
#include <signal.h>

#include "zeta_mdrobot_motor_control/mdrobot_motor_control.hpp"

#define CONTROL_MOTOR_SPEED_PERIOD      (1000/25)     //25hz. 40ms
#define DRIVE_INFO_PUBLISH_PERIOD       (1000/25)     //25hz. 40ms

#define DEBUG_BASIC
#define DEBUG_MC

#define VELCNT                           (1000/(CONTROL_MOTOR_SPEED_PERIOD))
#define MAXDIFF_VELLIN                   2.0    // m/s
#define MAXDIFF_VELANG                   2.0    // m/s

#define VELERRCNT                        CONTROL_MOTOR_SPEED_PERIOD  

#define MAX_LINEAR_VELOCITY              2.0             // m/s
#define MAX_ANGULAR_VELOCITY             3.0             // rad/s
#define LINEAR_X_MAX_VELOCITY            2.0

#define PI                               3.1415926535897932384626433832795
#define MATH_RAD2DEG                     57.2957795f
#define MATH_DEG2RAD                     0.0174532f

#define LEFT                             0
#define RIGHT                            1 

#define LINEAR                           0
#define ANGULAR                          1

#define GEARRATIO                         (30)

#define VELOCITY_UNIT                     2

#define WHEEL_NUM                         2
#define WHEEL_RADIUS                      0.0965        // meter
// #define WHEEL_RADIUS                      0.092
#define WHEEL_SEPARATION                  0.616         // meter
#define MOTOR_POLE                        10
#define TURNRATIO                         (MOTOR_POLE*3.0f)

#define DISTORPM                          ((60.0*GEARRATIO)/(2*PI*WHEEL_RADIUS))
#define RPMTODIS                          (1.0/DISTORPM)
#define PULSETODIST                       ((2*PI*WHEEL_RADIUS)/(TURNRATIO*GEARRATIO))

//#define _DEBUG_MC                         1
#define DEBUG_OM

nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_tf;
sensor_msgs::Imu imu_msg;
sensor_msgs::JointState joint_states;


float goal_velocity[VELOCITY_UNIT] = {0.0, 0.0};
float goal_velocity_from_cmd[VELOCITY_UNIT] = {0.0, 0.0};
bool teleop_flag = false;
float pre_velocity[VELOCITY_UNIT] = {0.0, 0.0};
float prev_wheel_velocity_cmd[2];
uint32_t tTime[5];
ros::Time current_time;
uint64_t current_offset;

int vellinearerrcnt = 0;
int velangerrcnt = 0;

float angval = 0.0f;


mdrobot_motor_control mdr_mc;

unsigned long time_now;
unsigned long step_time;
unsigned long prev_update_time;

std::string emergency_msg = "";


float left_motor_pos = 0.0f;
float right_motor_pos = 0.0f;
float left_motor_vel = 0.0f;
float right_motor_vel = 0.0f;

float v = 0.0f, w = 0.0f;

float pre_left_motor_pos = 0.0; 
float pre_right_motor_pos = 0.0; 
float pre_left_motor_vel = 0.0;
float pre_right_motor_vel = 0.0;

float delta_sl, delta_sr;
float delta_s = 0.0;

float theta = 0.0f;
float delta_theta = 0.0f;
float last_theta = 0.0f;

float odom_pose[3];
float odom_vel[3];

float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

float orientation[4];

float imu_theta, imu_prevtheta;
float imu_dtheta;

bool b_IMUFirst = true;

float constrain(float value, float min, float max);
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void imuSensorCallback(const sensor_msgs::Imu& imu_msg); //hong
void teleOPCallback(const std_msgs::Bool& bool_msg);
void emergencyCallback(const std_msgs::String::ConstPtr& msg);


void WheelControl(int* publish_rate);
void updateGoalVelocity(void);
void check_vel_safety(float *velocity);
ros::Time rosNow();
 bool controlMotor(float * value);
 void initOdom(void);
 bool calcOdometry(double diff_time);
 void updateOdometry(void);
 void updateTF(geometry_msgs::TransformStamped & odom_tf);
 void initJointStates(void);
 void updateJointStates(void);
 float rpm2MPS(short vel);
 float pulse2MPS(long pos);


int main (int argc, char** argv){
 
    int rate_b = 1000;
     
    ros::init(argc, argv, "zeta_mdrobot_motor_control_node");
    
    // spawn another thread
    boost::thread thread_b(WheelControl, &rate_b);

    thread_b.join(); 

#ifdef DEBUG_BASIC
    ROS_INFO_STREAM("End porcess!!!");
#endif

    while(ros::ok())
    {
        ;
    }

    return 0;
}

void WheelControl(int* publish_rate)
{

    uint32_t t;

    ros::NodeHandle node_obj;

    ros::Publisher odom_pub;

    ros::Publisher joint_states_pub;

    tf::TransformBroadcaster odom_broadcaster;

	ros::Rate loop_rate(*publish_rate);

    ros::Rate rev_delay(500);

	ros::Subscriber cmdvel_subscriber = node_obj.subscribe("cmd_vel",10,commandVelocityCallback);
    
    ros::Subscriber imu_sub = node_obj.subscribe("imu", 2000, imuSensorCallback); //hong

    ros::Subscriber teleop_subscriber = node_obj.subscribe("teleop",10,teleOPCallback);

    ros::Subscriber emergency_sub = node_obj.subscribe("emergency_stop",100,emergencyCallback);

	
    odom_pub = node_obj.advertise<nav_msgs::Odometry>("odom", 50);
    
    joint_states_pub = node_obj.advertise<sensor_msgs::JointState>("joint_states", 50);



	teleop_flag = false;
	
	// =======================================
    // Initialize MD Motor Controle Class
    // =======================================
	int ret = mdr_mc.InitSerial();
	
	if(ret == -1)
	{
#ifdef DEBUG_BASIC
    ROS_INFO_STREAM("Open error MDMC Serial Comm... End main process...");
#endif
		return; 
	}
	
    ROS_INFO_STREAM("Ready...");    

    // =======================================
    // Reset position.
    // =======================================
    mdr_mc.ResetPosition();

    ROS_INFO_STREAM("Reset position...");    

    initOdom();

    initJointStates();

     tTime[0] = (uint32_t)(ros::Time::now().toNSec()/1000000UL);

    prev_update_time = 0;

    b_IMUFirst = true; //hong


    unsigned long step_time;

    ros::Time stamp_now;

#if 1
    BYTE srcnt = 0;
#endif

    for(int i=0; i<2; i++)
        tTime[i] = 0;
    
    //mdr_mc.PNTMainDataBC(ON);
    //ROS_INFO_STREAM("On PNTMainData"); 
    
    mdr_mc.setStopStatus(0);

	while(ros::ok()){

        t = (uint32_t)(ros::Time::now().toNSec()/1000000UL);
        current_offset = ros::Time::now().toNSec();
        current_time   = ros::Time::now();


        if((t - tTime[0]) >= (DRIVE_INFO_PUBLISH_PERIOD - 5)) {

            // ==============================================
            // Send PNT MONITOR requesting command to MDMC 
            // ==============================================
            //mdr_mc.SendReqPIDData(PID_PNT_MONITOR);
            //mdr_mc.SendReqPIDData(PID_PNT_MAIN_DATA);
            
            if(mdr_mc.b_recvOK)
            {
                mdr_mc.b_recvOK = false;

                time_now = ros::Time::now().toNSec(); 
                step_time = time_now - prev_update_time;
                prev_update_time = time_now;

                short lm_vel, rm_vel;
                long lm_pos, rm_pos;

                // ==============================================
                // Get left and right velocity of robot. 
                // ==============================================
                mdr_mc.GetVelocityRMP(&lm_vel, &rm_vel);
                // ==============================================
                // Get left and right position of robot. 
                // ==============================================
                mdr_mc.GetMotorPosition(&lm_pos, &rm_pos);

                // left_motor_vel = rpm2MPS(lm_vel);
                // right_motor_vel = rpm2MPS(rm_vel);
                left_motor_vel = -1.0*(rpm2MPS(lm_vel));
                right_motor_vel = -1.0*(rpm2MPS(rm_vel));
                left_motor_pos = -1.0*pulse2MPS(lm_pos);
                right_motor_pos = -1.0*pulse2MPS(rm_pos);
                // left_motor_pos = pulse2MPS(lm_pos);
                // right_motor_pos = pulse2MPS(rm_pos);

#ifdef DEBUG_MC 
                //ROS_INFO("Velocity(%f, %f)", left_motor_vel, right_motor_vel);
                // ROS_INFO("[main] %f, %f, %f, %f", left_motor_vel, left_motor_pos, right_motor_vel, right_motor_pos);
#endif
                stamp_now = rosNow();
                
                // calculate odometry
                calcOdometry((double)((double)step_time/1000000000UL)); // dimension = [sec]

                // odometry
                updateOdometry();

                stamp_now = rosNow();
                odom.header.stamp = stamp_now;
                odom_pub.publish(odom);

                // odometry tf
                updateTF(odom_tf);
                stamp_now = rosNow();
                odom_tf.header.stamp = stamp_now;
                odom_broadcaster.sendTransform(odom_tf);

                updateJointStates();
                stamp_now = rosNow();
                joint_states.header.stamp = stamp_now;
                joint_states_pub.publish(joint_states); 
            }

            srcnt++;
            if(srcnt>25)
            {
                srcnt = 0;
                // ==============================================
                //  Check the normal operation status of MDMC
                // ==============================================
                if(mdr_mc.isMDMCRun() != ON)
                {
                    ROS_INFO_STREAM("ERROR : disconneted MD MC!!!");
                } else {
                    ROS_INFO_STREAM("Connecting MD MC....");
                }
            }

            tTime[0] = t;
        }

#if 1
        if((t - tTime[1]) >= (CONTROL_MOTOR_SPEED_PERIOD)) {
#ifdef _DEBUG_MC        
            ROS_INFO_STREAM("t-tTime[0]:" << (t - tTime[0]) << " CMSP Time : " << CONTROL_MOTOR_SPEED_PERIOD);
#endif            
           
            updateGoalVelocity();


            if (emergency_msg.find("stop") != std::string::npos)
            {
                mdr_mc.TorqueOff(ON);
            }
            else
            {
                
                check_vel_safety(goal_velocity);

                controlMotor(goal_velocity);
            }

            tTime[1] = t;
        }

#endif		
		loop_rate.sleep();

        ros::spinOnce();

	}

    // ==============================================
    // Send Torque Off command to MDMC 
    // ==============================================
    mdr_mc.TorqueOff(ON);
    
    ROS_INFO_STREAM("Torque off....");
}

void teleOPCallback(const std_msgs::Bool& bool_msg)
{
    if(bool_msg.data == true)
    {
        teleop_flag = true;
#ifdef _DEBUG_BASIC        
        ROS_INFO_STREAM("teleop is true.");
#endif        
    } else 
    {
        teleop_flag = false;
#ifdef _DEBUG_BASIC        
        ROS_INFO_STREAM("teleop is false.");
#endif        
    }
}

void imuSensorCallback(const sensor_msgs::Imu& imu_msg1) //hong
{
    imu_msg = imu_msg1;
	orientation[0] = imu_msg1.orientation.w;
	orientation[1] = 0;
	orientation[2] = 0;
	orientation[3] = imu_msg1.orientation.z;

#ifdef _DEBUG_IMU        
    ROS_INFO_STREAM("imu : " << orientation[0] << "," << orientation[1] << "," << orientation[2] << "," << orientation[3]);
#endif    

    imu_theta = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
                       0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3] );
    if(b_IMUFirst == true) {
        b_IMUFirst = false;
        imu_prevtheta = imu_theta;
    }
}

void emergencyCallback(const std_msgs::String::ConstPtr& msg)
{
    emergency_msg = msg->data;
}

void commandVelocityCallback(const geometry_msgs::Twist & cmd_vel_msg)
{

    goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
    goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

    goal_velocity_from_cmd[LINEAR]  = constrain(
                                      goal_velocity_from_cmd[LINEAR],
                                      (-1) * MAX_LINEAR_VELOCITY,
                                      MAX_LINEAR_VELOCITY
                                    );
    goal_velocity_from_cmd[ANGULAR] = constrain(
                                      goal_velocity_from_cmd[ANGULAR],
                                      (-1) * MAX_ANGULAR_VELOCITY,
                                      MAX_ANGULAR_VELOCITY
                                    );
#ifdef _DEBUG_MC        
    ROS_INFO_STREAM("cmdLS:" << goal_velocity_from_cmd[LINEAR] << " cmdAS:" << goal_velocity_from_cmd[ANGULAR]);
	//ROS_INFO_STREAM("cmd Linear Speed:" << goal_velocity_from_cmd[LINEAR]);
	//ROS_INFO_STREAM("cmd Angular Speed:" << goal_velocity_from_cmd[ANGULAR]);
#endif      
   
}


void updateGoalVelocity(void)
{
    if (emergency_msg.find("stop") == std::string::npos)
    {
        // Recieve goal velocity through ros messages
        goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
        goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
    }
    else
    {
        goal_velocity[LINEAR]  = 0;
        goal_velocity[ANGULAR] = 0;
        mdr_mc.BreakOff(1);

    }

#ifdef _DEBUG_MC        
	//ROS_INFO_STREAM("GLS:" << goal_velocity[LINEAR] << " GAS:" << goal_velocity[ANGULAR]);
#endif     
}

void check_vel_safety(float *velocity)
{

#if 1    
    if(fabs(pre_velocity[LINEAR]-velocity[LINEAR])>MAXDIFF_VELLIN) {
        if(vellinearerrcnt>VELERRCNT) {
            vellinearerrcnt = 0;

            velocity[LINEAR] = pre_velocity[LINEAR];

#ifdef DEBUG_MC
            ROS_INFO_STREAM("Error large linear velocity");    
#endif
        } else {
            vellinearerrcnt++;
        }
    }

    if(fabs(pre_velocity[ANGULAR]-velocity[ANGULAR])>MAXDIFF_VELANG) {
        if(velangerrcnt>VELERRCNT) {
            velangerrcnt = 0;

            velocity[ANGULAR] = pre_velocity[ANGULAR];

#ifdef DEBUG_MC
            ROS_INFO_STREAM("Error large angular velocity");    
#endif            

        } else {
            velangerrcnt++;
        }
    }
#endif

    pre_velocity[LINEAR] = velocity[LINEAR];
    pre_velocity[ANGULAR] = velocity[ANGULAR];

} 

ros::Time rosNow()
{
    uint32_t sec, nsec;

    
    uint64_t _micros = ros::Time::now().toNSec() - current_offset;


    sec  = (uint32_t)(_micros / 1000000) + current_time.sec;
    nsec = (uint32_t)(_micros % 1000000) + 1000 * (current_time.nsec / 1000);

    if (nsec >= 1e9) {
        sec++,
        nsec--;
    }
    return ros::Time(sec, nsec);
 }
 
 bool controlMotor(float * value)
{
    float wheel_velocity_cmd[2];

    wheel_velocity_cmd[LEFT]  = value[LINEAR] + (value[ANGULAR] * WHEEL_SEPARATION / 2.0f);
    wheel_velocity_cmd[RIGHT] = value[LINEAR] - (value[ANGULAR] * WHEEL_SEPARATION / 2.0f);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT], -LINEAR_X_MAX_VELOCITY, LINEAR_X_MAX_VELOCITY);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT], -LINEAR_X_MAX_VELOCITY, LINEAR_X_MAX_VELOCITY);

    wheel_velocity_cmd[LEFT] = -1.0*wheel_velocity_cmd[LEFT]*DISTORPM;
    wheel_velocity_cmd[RIGHT] = 1.0*wheel_velocity_cmd[RIGHT]*DISTORPM;

    //if((prev_wheel_velocity_cmd[LEFT] != wheel_velocity_cmd[LEFT]) || (prev_wheel_velocity_cmd[RIGHT] != wheel_velocity_cmd[RIGHT])) {
		
        // ===================================
        // Set velocity of mobile robot
        // ===================================
		mdr_mc.SetVelocity((short)wheel_velocity_cmd[LEFT], (short)wheel_velocity_cmd[RIGHT]);

        prev_wheel_velocity_cmd[LEFT] = wheel_velocity_cmd[LEFT];
        prev_wheel_velocity_cmd[RIGHT] = wheel_velocity_cmd[RIGHT];

#ifdef _DEBUG_MC        
        ROS_INFO_STREAM("CtrlVel(" << wheel_velocity_cmd[LEFT] << "," << wheel_velocity_cmd[RIGHT] << ")");
#endif        
    //}

    return true;
}

float constrain(float value, float min, float max)
{
    if(value > max) return max;
    if(value < min) return min;

    // ignore very small velocity
    if(fabs(value) < 0.001f)
        return 0.0f;

    return value;
}

bool calcOdometry(double diff_time)
{

    if (diff_time == 0)
        return false;

    //wheel_vel_l = -left_motor_vel;
    //wheel_vel_r = right_motor_vel;

#if 0
    delta_sl = -left_motor_pos - pre_left_motor_pos;
    delta_sr = right_motor_pos - pre_right_motor_pos;

    delta_theta = (delta_sr - delta_sl)/WHEEL_SEPARATION;

    theta += delta_theta;
#endif

	// 2019.06.22
#if 0    
	theta = atan2f(imu_msg.orientation.x * imu_msg.orientation.y + imu_msg.orientation.w * imu_msg.orientation.z,
		0.5f - imu_msg.orientation.y * imu_msg.orientation.y - imu_msg.orientation.z * imu_msg.orientation.z );

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("theta:" << theta);
#endif
#endif

#if 0
    // 2019.07.24
    left_motor_vel = (-left_motor_pos - pre_left_motor_pos) / diff_time;
    right_motor_vel = (right_motor_pos - pre_right_motor_pos) / diff_time;


    pre_left_motor_pos = -left_motor_pos;
    pre_right_motor_pos = right_motor_pos;

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("calOdom diff time:" << diff_time);
#endif
#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("cal vel(" << left_motor_vel << "," << right_motor_vel << ")");
#endif
    //delta_vl = -left_motor_vel - pre_left_motor_vel;
    //delat_vr = right_motor_vel - pre_right_motor_vel;
#endif

    v = (-left_motor_vel + right_motor_vel) / 2.0;
    w = (right_motor_vel - (-1.0*left_motor_vel))/ WHEEL_SEPARATION;
    //w = (-left_motor_vel - right_motor_vel)/ WHEEL_SEPARATION;

    pre_left_motor_vel  = -left_motor_vel;
    pre_right_motor_vel = right_motor_vel;

    delta_s = (diff_time * v) ;

    //delta_theta = w - odom_pose[2];

    pre_left_motor_pos = -left_motor_pos;
    pre_right_motor_pos = right_motor_pos;

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("odometry(v,w):(" << v << "," << w << ")");
    //ROS_INFO_STREAM("odometry(ds, dth):(" << delta_s << "," << delta_theta << ")");
#endif    
    // must apply delta_theta to imu sensor value
    //delta_theta = theta - last_theta;

    // New algorithm
    
    //hong
    imu_dtheta = (imu_theta - imu_prevtheta);
    // imu_dtheta = -1.0*(imu_theta - imu_prevtheta);

    //if(fabs(imu_theta) > 

    //odom_pose[2]  += imu_dtheta;
    //if(fabs(imu_dtheta)>IMU_MINTHETA)
    if(fabs(imu_dtheta)>0.0001)   
        odom_pose[2]  += imu_dtheta;

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("odometry(ds, th, dth):(" << delta_s << "," << imu_theta << "," << imu_dtheta << ")");
#endif    


    angval = MATH_RAD2DEG*odom_pose[2];

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("odom_pos2:" << odom_pose[2] << "  imu_dtheta:" << imu_dtheta);
#endif
#ifdef _DEBUG_OM            
    ROS_INFO_STREAM("heading angle:(" << angval << ")");
#endif        


    // compute odometric pose
    odom_pose[0]  += delta_s * cos(odom_pose[2] + (imu_dtheta / 2.0));
    odom_pose[1]  += delta_s * sin(odom_pose[2] + (imu_dtheta / 2.0));

    //odom_pose[0]  += (delta_s * cos(odom_pose[2] + (delta_theta / 2.0)) - CENTER_DIFF*cos(odom_pose[2] + (imu_dtheta / 2.0)) );
    //odom_pose[1]  += (delta_s * sin(odom_pose[2] + (delta_theta / 2.0))  - CENTER_DIFF*sin(odom_pose[2] + (imu_dtheta / 2.0)) );

    //odom_pose[0]  += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    //odom_pose[1]  += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));

    // OK...
    //odom_pose[0]  += (delta_s * cos(odom_pose[2] + (imu_dtheta / 2.0)) - CENTER_DIFF*sin(odom_pose[2] + (imu_dtheta / 2.0)) );
    //odom_pose[1]  += (delta_s * sin(odom_pose[2] + (imu_dtheta / 2.0))  + CENTER_DIFF*cos(odom_pose[2] + (imu_dtheta / 2.0)) );

    //odom_pose[0]  += (delta_s * cos(odom_pose[2] + (delta_theta / 2.0)) - CENTER_DIFF*sin(odom_pose[2] + (delta_theta / 2.0)) );
    //odom_pose[1]  += (delta_s * sin(odom_pose[2] + (delta_theta / 2.0))  + CENTER_DIFF*cos(odom_pose[2] + (delta_theta / 2.0)) );

    //odom_pose[0]  += (delta_s * cos(odom_pose[2] + (delta_theta / 2.0)) + CENTER_DIFF*cos(odom_pose[2] + (delta_theta / 2.0)) );
    //odom_pose[1]  += (delta_s * sin(odom_pose[2] + (delta_theta / 2.0))  + CENTER_DIFF*sin(odom_pose[2] + (delta_theta / 2.0)) );



    //odom_pose[2]  += delta_theta;

    // compute odometric instantaneouse velocity
    odom_vel[0] = v;
    // odom_vel[0] = -1.0*v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    //last_theta = theta;

#ifdef _DEBUG_OM        
    ROS_INFO_STREAM("odom_pos (" << odom_pose[0] << "," << odom_pose[1] << "," << odom_pose[2] << ")");
    //ROS_INFO_STREAM("odom_vel:(" << odom_vel[0] << "," << odom_vel[1] << "," << odom_vel[2] << ")");
  
    //ROS_INFO_STREAM("imu dtheta:(" << imu_dtheta << ")");
    //ROS_INFO_STREAM("encoder dtheta:(" << delta_theta << ")");

    //ROS_INFO_STREAM("imu theta (" << imu_theta << ")");
    //ROS_INFO_STREAM("encoder theta (" << theta << ")");
#endif

    imu_prevtheta = imu_theta;

    //hong

    return true;
}

void initOdom(void)
{

    b_IMUFirst = true;

    imu_theta = 0.0f;
    imu_prevtheta = 0.0f;
    imu_dtheta = 0.0f;


    for (int index = 0; index < 3; index++)
    {
        odom_pose[index] = 0.0;
        odom_vel[index]  = 0.0;
    }


    odom.pose.pose.position.x    = 0.0;
    odom.pose.pose.position.y    = 0.0;
    odom.pose.pose.position.z    = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;

    odom.twist.twist.linear.x    = 0.0;
    odom.twist.twist.angular.z   = 0.0;
}

void updateOdometry(void)
{
    odom.header.frame_id       = "odom";
    odom.child_frame_id        = "base_link";

    odom.pose.pose.position.x  = odom_pose[0];
    odom.pose.pose.position.y  = odom_pose[1];
    odom.pose.pose.position.z  = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.linear.y  = odom_vel[1];
    odom.twist.twist.angular.z = odom_vel[2];
}

void updateTF(geometry_msgs::TransformStamped & odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
}

void initJointStates(void)
{
     joint_states.header.frame_id = "base_link";
 
    joint_states.name.push_back("wheel_left_joint");
    joint_states.name.push_back("wheel_right_joint");

    unsigned int n = joint_states.name.size();
    joint_states.position.resize(n);
    joint_states.velocity.resize(n);
    joint_states.effort.resize(n);
}

void updateJointStates(void)
{
    joint_states_pos[LEFT] = pre_left_motor_pos;
    joint_states_pos[RIGHT] = pre_right_motor_pos;

    joint_states_vel[LEFT] = pre_left_motor_vel;
    joint_states_vel[RIGHT] = pre_right_motor_vel;

    for(int i=0; i<WHEEL_NUM; i++) {
        joint_states.position[i] = joint_states_pos[i];
        joint_states.velocity[i] = joint_states_vel[i];
        joint_states.effort[i] = joint_states_eff[i];
    }
}

float rpm2MPS(short vel) 
{  
    // Conversion from RPM to m/s


       return (float)(vel * RPMTODIS);

}

float pulse2MPS(long pos)
{
   // Conversion from Pulse to m

    if(pos > 0)
        return (float)(pos * PULSETODIST);
    else
        return (float)(-pos * PULSETODIST);
}