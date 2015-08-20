/*
 * Autor: Steve Ataucuri
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mindwave_msgs/Mindwave.h>

/*
  This class allows to teleoperate a Quaqcopter robot with
  the Mindwave headset
*/

class QuadcopterTeleop
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
  /*
    Constructor method's that subscribe to /mindwave topic   
  */
  QuadcopterTeleop()
  { 
    vel.linear.x = vel.linear.y = vel.angular.z = 0;
    x_vel = y_vel = z_vel = yaw_vel = 0;

    ros::NodeHandle n("~");

    sub = nh.subscribe<mindwave_msgs::Mindwave>("/mindwave", 1, &QuadcopterTeleop::mindwaveCallback, this);
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

  /*
    This method publishes a rostopic /cmd_vel with angular and linealvelocities 
    to move a quadcopter. When the user concentrate or meditate can move the 
    robot.
    This method moves the robot with a threshold value in the launch file

    Param:
      msg : ROS message with meditation and attention values  

  */  
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
