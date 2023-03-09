#include "autonomous_navigation/zetabotControl.h"


ZetabotControl::ZetabotControl()
{
	ROS_INFO("### Zetabot control ###");	
  teleop_msg.data = false;
  cmd_pub_cnt = 0;
  
	cmd_srv = nh_.advertiseService("/zetabot/command", &ZetabotControl::cmdInfoCb, this);
  
  local_cost_client =  nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/local_costmap/set_parameters");
  dwa_plan_client =  nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/DWAPlannerROS/set_parameters");
  clear_costmap_client =  nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  teleopPublisher = nh_.advertise<std_msgs::Bool>("/teleop", 1000);

  velSubscriber = nh_.subscribe("/cmd_vel", 1000, &ZetabotControl::getVelCb, this);
  mapSubscriber = nh_.subscribe("/map_metadata", 1000, &ZetabotControl::mapInfoCb, this);
}

ZetabotControl::~ZetabotControl()
{

}


void ZetabotControl::getVelCb(geometry_msgs::Twist cmd_vel)
{
  current_vel = cmd_vel;
}

void ZetabotControl::mapInfoCb(nav_msgs::MapMetaData map_data)
{
  g_map_data = map_data;
}

bool ZetabotControl::cmdInfoCb(zetabank_msgs::CmdInfo::Request  &req, zetabank_msgs::CmdInfo::Response &res)
{
	ROS_INFO_STREAM("Command Information callback func! ");
  zetanav.stop(stop);
  
	this->current_cmdInfo.request.command = req.command;
	this->current_cmdInfo.request.speed = req.speed;
	this->current_cmdInfo.request.moveto_x = req.moveto_x;
	this->current_cmdInfo.request.moveto_y = req.moveto_y;
	this->current_cmdInfo.request.moveto_theta = req.moveto_theta;
	this->current_cmdInfo.request.setled = req.setled;

  ROS_INFO_STREAM("Callback Command: " << this->current_cmdInfo.request);
  res.result = true;
  return true;
}

void ZetabotControl::updataDynCfg( string str)
{
  if(str == "slow")
  {  
    system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.2");    
    ROS_INFO("##### Slow Speed! ##### ");
    //updataSlowMode(); 

  }  
  else if(str == "normal")
  {
    system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.4"); 
    ROS_INFO("#####  Normal Speed! ##### ");
    //updataNormalMode(); 
    
    }
  else if(str == "fast")
  {
    ROS_INFO("#####  Fast Speed! ##### ");
    updataFastMode(); 
  }
  else
  {
    system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.7");    
    ROS_INFO("##### Unknown Speed! ##### ");
    //updataNormalMode(); 
    //system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 1.0");  
  }
}


void ZetabotControl::doAutoNav()
{
  while(ros::ok())
  {
      cmd_pub_cnt++;
      if(cmd_pub_cnt >= CMD_STOP_PUB_CNT)
        cmd_pub_cnt = CMD_STOP_PUB_CNT;

      //ROS_INFO_STREAM("Input Command: " << this->current_cmdInfo.request.command);

      if(this->current_cmdInfo.request.command == "moveto" )
      {  
        // Integration integer with responce
        std::ostringstream os;
        os << "Moveto:" << current_cmdInfo.request.moveto_x << ", " << current_cmdInfo.request.moveto_y << ", "  << current_cmdInfo.request.moveto_theta;
        ros::param::set("/zetabot/status/response", os.str());   

        geometry_msgs::TransformStamped transformStamped;
        transformStamped = calcPixel2Pos(current_cmdInfo.request.moveto_x, current_cmdInfo.request.moveto_y, current_cmdInfo.request.moveto_theta);
        execMoveTo(transformStamped);      
      }	
      else if(this->current_cmdInfo.request.command == "stop")
      {
        execStop();
        ros::param::set("/zetabot/status/response", "stop");  
      }
      else if(this->current_cmdInfo.request.command == "teleop")
      {
        // Integration integer with responce
        std::ostringstream os;
        os << "teleop: "<< this->current_cmdInfo.request.moveto_x << ", " << this->current_cmdInfo.request.moveto_y << ", "  << current_cmdInfo.request.moveto_theta; 
        ros::param::set("/zetabot/status/response", os.str());  

        execTeleop(this->current_cmdInfo.request.moveto_x , this->current_cmdInfo.request.moveto_theta );
      }
      else if(this->current_cmdInfo.request.command == "success" )
      {              
        ros::param::set("/zetabot/status/response", "success"); 
      }
      else
      {
        execStop();
        ros::param::set("/zetabot/status/response", this->current_cmdInfo.request.command);  
        //ROS_INFO("Command is wrong");        
      }
      ros::spinOnce();
  }  
}

geometry_msgs::TransformStamped ZetabotControl::calcPixel2Pos(float goal_x, float goal_y, float goal_theta)
{
  geometry_msgs::TransformStamped transformStamped;

  //set a tf inheriting map.
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "goal";
  // origin(0, 0) = (g_map_data.origin.position.x,  g_map_data.origin.position.y + g_map_data.height *g_map_data.resolution)
  transformStamped.transform.translation.x =  g_map_data.origin.position.x + goal_x * g_map_data.resolution ;
  transformStamped.transform.translation.y =  g_map_data.origin.position.y + g_map_data.height *g_map_data.resolution - goal_y * g_map_data.resolution ;
  transformStamped.transform.translation.z = 0.0;

  q.setRPY(0, 0, goal_theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  
  transformStamped.header.stamp = ros::Time::now();

  return transformStamped;
}

void ZetabotControl::execMoveTo(geometry_msgs::TransformStamped goal_transformStamped)
{        
  // To set ultrasonic safety mode to the OpenCR
  teleop_msg.data = false;
  teleopPublisher.publish(teleop_msg);

  // Clear costmap to make plan trajectory
  if (clear_costmap_client.call(empty_srv))
  {
    ROS_INFO_STREAM("Local costmap clear ");
    ros::Duration(1).sleep();
  }
  else
    ROS_ERROR("Failed to clear costmap");

  // compare previous speed mode with speed mode such as slow, normal, fast
  if(this->current_cmdInfo.request.speed != pre_speed.data)
  {
    ROS_INFO_STREAM("Load the diffent speed mode parameters ");
    current_speed.data = this->current_cmdInfo.request.speed;
    updataDynCfg( this->current_cmdInfo.request.speed);
    pre_speed.data = current_speed.data;

    // Change of speed can needs to time for loading parameters
    ros::Duration(1).sleep();
  }

  // Set the goal for moving and receive the return value if reach the goal
  if(zetanav.setGoal(goal_transformStamped) && zetanav.sendgoal_flgs == true)
  {
    this->current_cmdInfo.request.command = "success" ;
  }
  else if(zetanav.sendgoal_flgs == true)
  {          
    // Fail to plan the trajectory
    execStop();
    ROS_ERROR("### Failed to plan, Stop the robot ###");  
  }  
  else
  {          
    // if cmdInfoCb execute, setGoal func occurs problem, So sendgoal_flgs is added in Zetanavigation class 
    execStop();
    ROS_INFO("### Suddenly Stop ###");  
  }  
}

void ZetabotControl::execStop()
{
  if( cmd_pub_cnt >= CMD_STOP_PUB_CNT)
  {
    zetanav.stop(stop);
    cmd_pub_cnt = 0;
  }
}

void ZetabotControl::execTeleop(float trans_vel, float angular_vel)
{
  // To use teleoperation
  teleop_msg.data = true;

  // To send velocity
  geometry_msgs::Twist vel;

  vel.linear.x = trans_vel;
  vel.angular.z = angular_vel;

  if( cmd_pub_cnt >= CMD_STOP_PUB_CNT)
  {
    teleopPublisher.publish(teleop_msg);
    zetanav.teleop(vel);
    cmd_pub_cnt = 0;
  }

  teleop_msg.data = false;
  //system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 1.0");    
}


void ZetabotControl::updataSlowMode( )
{
    updataSlowLocalMap();
    updataSlowDWAPlan();
}

void ZetabotControl::updataSlowLocalMap( )
{
  std::vector<dynamic_reconfigure::IntParameter> vec_int;
  std::vector<dynamic_reconfigure::StrParameter> vec_str;
  std::vector<dynamic_reconfigure::DoubleParameter> vec_db;
    
  dyn_int.name = "width";
  ros::param::get("slow/local_costmap/width", dyn_int.value);		
  vec_int.push_back(dyn_int);
  dyn_int.name = "height";
  ros::param::get("slow/local_costmap/height", dyn_int.value);	
  vec_int.push_back(dyn_int);


  dyn_db.name = "transform_tolerance";
  ros::param::get("slow/local_costmap/transform_tolerance", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "update_frequency";
  ros::param::get("slow/local_costmap/update_frequency", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "publish_frequency";
  ros::param::get("slow/local_costmap/publish_frequency", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "resolution";
  ros::param::get("slow/local_costmap/resolution", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "origin_x";
  ros::param::get("slow/local_costmap/origin_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "origin_y";
  ros::param::get("slow/local_costmap/origin_y", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_cfg.ints = vec_int;
  dyn_cfg.strs = vec_str;
  dyn_cfg.doubles = vec_db;

  vec_int.clear();
  vec_str.clear();
  vec_db.clear();

  srv.request.config = dyn_cfg;

  if (local_cost_client.call(srv))
    ROS_INFO_STREAM("Local costmap Call " << dyn_cfg );
  else
    ROS_ERROR("Failed to call service add_two_ints");
}

void ZetabotControl::updataSlowDWAPlan( )
{
  std::vector<dynamic_reconfigure::IntParameter> vec_int;
  std::vector<dynamic_reconfigure::DoubleParameter> vec_db;
  std::vector<dynamic_reconfigure::StrParameter> vec_str;

  dyn_int.name = "vx_samples";
  ros::param::get("slow/DWAPlannerROS/vx_samples", dyn_int.value);	
  vec_int.push_back(dyn_int);
  dyn_int.name = "vth_samples";
  ros::param::get("slow/DWAPlannerROS/vth_samples", dyn_int.value);	
  vec_int.push_back(dyn_int);

  dyn_db.name = "acc_lim_x";
  ros::param::get("slow/DWAPlannerROS/acc_lim_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "acc_lim_theta";
  ros::param::get("slow/DWAPlannerROS/acc_lim_theta", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_vel_x";
  ros::param::get("slow/DWAPlannerROS/max_vel_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_vel_x";
  ros::param::get("slow/DWAPlannerROS/min_vel_x", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_trans_vel";
  ros::param::get("slow/DWAPlannerROS/max_trans_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_trans_vel";
  ros::param::get("slow/DWAPlannerROS/min_trans_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_rot_vel";
  ros::param::get("slow/DWAPlannerROS/max_rot_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_rot_vel";
  ros::param::get("slow/DWAPlannerROS/min_rot_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "sim_time";
  ros::param::get("slow/DWAPlannerROS/sim_time", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_cfg.ints = vec_int;
  dyn_cfg.doubles = vec_db;
  dyn_cfg.strs = vec_str;

  vec_int.clear();
  vec_str.clear();
  vec_db.clear();

  srv.request.config = dyn_cfg;


  if (dwa_plan_client.call(srv))
    ROS_INFO_STREAM("Local costmap Call " << dyn_cfg );
  else
    ROS_ERROR("Failed to call service add_two_ints");
}

void ZetabotControl::updataNormalMode( )
{
    updataNormalLocalMap();
    updataNormalDWAPlan();

}
void ZetabotControl::updataNormalLocalMap( )
{
  std::vector<dynamic_reconfigure::IntParameter> vec_int;
  std::vector<dynamic_reconfigure::StrParameter> vec_str;
  std::vector<dynamic_reconfigure::DoubleParameter> vec_db;
    
  dyn_int.name = "width";
  ros::param::get("normal/local_costmap/width", dyn_int.value);		
  vec_int.push_back(dyn_int);
  dyn_int.name = "height";
  ros::param::get("normal/local_costmap/height", dyn_int.value);	
  vec_int.push_back(dyn_int);


  dyn_db.name = "transform_tolerance";
  ros::param::get("normal/local_costmap/transform_tolerance", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "update_frequency";
  ros::param::get("normal/local_costmap/update_frequency", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "publish_frequency";
  ros::param::get("normal/local_costmap/publish_frequency", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "resolution";
  ros::param::get("normal/local_costmap/resolution", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "origin_x";
  ros::param::get("normal/local_costmap/origin_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "origin_y";
  ros::param::get("normal/local_costmap/origin_y", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_cfg.ints = vec_int;
  dyn_cfg.strs = vec_str;
  dyn_cfg.doubles = vec_db;

  vec_int.clear();
  vec_str.clear();
  vec_db.clear();

  srv.request.config = dyn_cfg;

  if (local_cost_client.call(srv))
    ROS_INFO_STREAM("Local costmap Call " << dyn_cfg );
  else
    ROS_ERROR("Failed to call service add_two_ints");
}

void ZetabotControl::updataNormalDWAPlan( )
{
  std::vector<dynamic_reconfigure::IntParameter> vec_int;
  std::vector<dynamic_reconfigure::DoubleParameter> vec_db;

  dyn_int.name = "vx_samples";
  ros::param::get("normal/DWAPlannerROS/vx_samples", dyn_int.value);	
  vec_int.push_back(dyn_int);
  dyn_int.name = "vth_samples";
  ros::param::get("normal/DWAPlannerROS/vth_samples", dyn_int.value);	
  vec_int.push_back(dyn_int);


  dyn_db.name = "acc_lim_x";
  ros::param::get("normal/DWAPlannerROS/acc_lim_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "acc_lim_theta";
  ros::param::get("normal/DWAPlannerROS/acc_lim_theta", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_vel_x";
  ros::param::get("normal/DWAPlannerROS/max_vel_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_vel_x";
  ros::param::get("normal/DWAPlannerROS/min_vel_x", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_trans_vel";
  ros::param::get("normal/DWAPlannerROS/max_trans_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_trans_vel";
  ros::param::get("normal/DWAPlannerROS/min_trans_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_rot_vel";
  ros::param::get("normal/DWAPlannerROS/max_rot_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_rot_vel";
  ros::param::get("normal/DWAPlannerROS/min_rot_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "sim_time";
  ros::param::get("normal/DWAPlannerROS/sim_time", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_cfg.ints = vec_int;
  dyn_cfg.doubles = vec_db;
  srv.request.config = dyn_cfg;

  vec_int.clear();
  vec_db.clear();


  if (dwa_plan_client.call(srv))
    ROS_INFO_STREAM("Local costmap Call " << dyn_cfg );
  else
    ROS_ERROR("Failed to call service add_two_ints");

}

void ZetabotControl::updataFastMode( )
{
    updataFastLocalMap();
    updataFastDWAPlan();

}
void ZetabotControl::updataFastLocalMap( )
{
  std::vector<dynamic_reconfigure::IntParameter> vec_int;
  std::vector<dynamic_reconfigure::StrParameter> vec_str;
  std::vector<dynamic_reconfigure::DoubleParameter> vec_db;
    
  dyn_int.name = "width";
  ros::param::get("fast/local_costmap/width", dyn_int.value);		
  vec_int.push_back(dyn_int);
  dyn_int.name = "height";
  ros::param::get("fast/local_costmap/height", dyn_int.value);	
  vec_int.push_back(dyn_int);

  dyn_db.name = "transform_tolerance";
  ros::param::get("fast/local_costmap/transform_tolerance", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "update_frequency";
  ros::param::get("fast/local_costmap/update_frequency", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "publish_frequency";
  ros::param::get("fast/local_costmap/publish_frequency", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "resolution";
  ros::param::get("fast/local_costmap/resolution", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "origin_x";
  ros::param::get("fast/local_costmap/origin_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "origin_y";
  ros::param::get("fast/local_costmap/origin_y", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_cfg.ints = vec_int;
  dyn_cfg.strs = vec_str;
  dyn_cfg.doubles = vec_db;
  vec_int.clear();
  vec_str.clear();
  vec_db.clear();

  srv.request.config = dyn_cfg;

  if (local_cost_client.call(srv))
    ROS_INFO_STREAM("Local costmap Call " << dyn_cfg );
  else
    ROS_ERROR("Failed to call service add_two_ints");
}
void ZetabotControl::updataFastDWAPlan()
{
  std::vector<dynamic_reconfigure::IntParameter> vec_int;
  std::vector<dynamic_reconfigure::DoubleParameter> vec_db;

  dyn_int.name = "vx_samples";
  ros::param::get("fast/DWAPlannerROS/vx_samples", dyn_int.value);	
  vec_int.push_back(dyn_int);
  dyn_int.name = "vth_samples";
  ros::param::get("fast/DWAPlannerROS/vth_samples", dyn_int.value);	
  vec_int.push_back(dyn_int);


  dyn_db.name = "acc_lim_x";
  ros::param::get("fast/DWAPlannerROS/acc_lim_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "acc_lim_theta";
  ros::param::get("fast/DWAPlannerROS/acc_lim_theta", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_vel_x";
  ros::param::get("fast/DWAPlannerROS/max_vel_x", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_vel_x";
  ros::param::get("fast/DWAPlannerROS/min_vel_x", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_trans_vel";
  ros::param::get("fast/DWAPlannerROS/max_trans_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_trans_vel";
  ros::param::get("fast/DWAPlannerROS/min_trans_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "max_rot_vel";
  ros::param::get("fast/DWAPlannerROS/max_rot_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);
  dyn_db.name = "min_rot_vel";
  ros::param::get("fast/DWAPlannerROS/min_rot_vel", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_db.name = "sim_time";
  ros::param::get("fast/DWAPlannerROS/sim_time", dyn_db.value);	
  vec_db.push_back(dyn_db);

  dyn_cfg.ints = vec_int;
  dyn_cfg.doubles = vec_db;
  srv.request.config = dyn_cfg;

  vec_int.clear();
  vec_db.clear();


  if (dwa_plan_client.call(srv))
    ROS_INFO_STREAM("Local costmap Call " << dyn_cfg );
  else
    ROS_ERROR("Failed to call service add_two_ints");

}
