#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

#include <mindwave_execute_trajectory/ExecTraj.h>
#include <mindwave_msgs/Mindwave.h>



using namespace std;

const std::string trajFiles[2]={"home_pose.txt","end_pose.txt"};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "executor_node");

  // Start the service
  ros::NodeHandle nh;

  ros::ServiceClient traj_client_ = nh.serviceClient<mindwave_execute_trajectory::ExecTraj>("/execute_trajectory");
  mindwave_execute_trajectory::ExecTraj trajectory;

  int i = 0;

  while (ros::ok())
  {
    
      trajectory.request.file = ros::package::getPath("mindwave_execute_trajectory") + "/config/" + trajFiles[i];
      ROS_INFO("Executing trajectory %s", trajFiles[i].c_str());
      
      if (!traj_client_.call(trajectory))
      {
          ROS_ERROR ("Failed to execute [%s] trajectory", trajFiles[i].c_str());
          break;
      }

      i = (i + 1) % 2;

      ros::spinOnce();
      usleep(100000);
  }

  return 0;
}

/*

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mindwave_msgs/Mindwave.h>

class ArmTeleop
{
  private:
 
  geometry_msgs::Twist vel;

  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

  int x_threshold, y_threshold, z_threshold;
  int m_threshold;
  double x_vel,y_vel,z_vel, yaw_vel;  double default_yaw;

  public: 
  ArmTeleop()
  { 
    vel.linear.x = vel.linear.y = vel.angular.z = 0;
    x_vel = y_vel = z_vel = yaw_vel = 0;

    ros::NodeHandle n("~");

    sub = nh.subscribe<mindwave_msgs::Mindwave>("/mindwave", 1, &ArmTeleop::mindwaveCallback, this);
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    n.param<int>("x_attention_threshold", x_threshold, 85);
    n.param<int>("y_attention_threshold", y_threshold, 65);
    n.param<int>("z_attention_threshold", z_threshold, 50);
    n.param<int>("meditation_threshold", m_threshold, 100);

    n.param<double>("x_vel", x_vel, 2);
    n.param<double>("y_vel", y_vel, 2);
    n.param<double>("z_vel", z_vel, 2);
    n.param<double>("yaw_vel", yaw_vel, 90.0 * M_PI/ 180.0);

  }
  
  ~QuadcopterTeleop()   { }
  
  void mindwaveCallback(const mindwave_msgs::Mindwave msg){
    
    vel.linear.x = vel.linear.y = vel.angular.z = 0;

    if(msg.attention > x_threshold)
      vel.linear.x = x_vel;
        
    if(msg.attention > y_threshold)
      vel.linear.y = y_vel;

    if(msg.attention > z_threshold)
      vel.linear.z = z_vel;
    else
      vel.linear.z = 0; 
    
    if(msg.meditation > m_threshold)  
      vel.angular.y = yaw_vel;
 

    pub.publish(vel);
    
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadcopter_teleop_mindwave");

  QuadcopterTeleop quad;
  
  ros::spin();
  
  return(0);
}
*/