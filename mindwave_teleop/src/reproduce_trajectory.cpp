#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

//#include <robotiq_s_model_articulated_msgs/SModelRobotInput.h>
//#include <robotiq_s_model_articulated_msgs/SModelRobotOutput.h>
#include <robotiq_msgs/SModelRobotInput.h>
#include <robotiq_msgs/SModelRobotOutput.h>


#include <mindwave_msgs/Mindwave.h>
#include <mindwave_execute_trajectory/ExecTraj.h>


using namespace std;

/*
  This class allows to teleoperate the robot arm with
  the Mindwave headset
*/
class ArmTeleop
{
  private:
 
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::ServiceClient traj_client;
  mindwave_execute_trajectory::ExecTraj trajectory;

  robotiq_msgs::SModelRobotOutput cmd; 

  int m_threshold;
  int a_threshold;
  bool picked;
  bool positioned;

  string trajFiles[2];
  string pick_traj, place_traj;

  public: 
  ArmTeleop()
  { 
    
    ros::NodeHandle n("~");

    n.param<int>("meditation_threshold", m_threshold, 80);
    n.param<int>("attention_threshold", a_threshold, 50);
    n.getParam("pick", pick_traj);
    n.getParam("place", place_traj);

    picked = false;
    positioned = false;

    if (pick_traj != "" || place_traj !="")
    { 
      trajFiles[0]= pick_traj;
      trajFiles[1]= place_traj;

      sub = nh.subscribe<mindwave_msgs::Mindwave>("/mindwave", 1, &ArmTeleop::mindwaveCallback, this);
      pub = nh.advertise<robotiq_msgs::SModelRobotOutput>("left_hand/command", 10);
      
      // Start the service
      ros::NodeHandle nh;

      traj_client = nh.serviceClient<mindwave_execute_trajectory::ExecTraj>("/execute_trajectory");

      control_gripper(true);

    } else{
      ROS_INFO("Not trajectory file found, please review that.");
    } 

  }
  
  ~ArmTeleop()   { }
  
  /*
    This method moves the arm to desired position calling to a service to execute
    a predefined trajectory. When the user concentrate or meditate can move the 
    arm

    Param:
      msg : ROS message with meditation and attention values  

  */
  void mindwaveCallback(const mindwave_msgs::Mindwave msg){
    
    string state = trajectory.response.state;

    if(msg.attention >= a_threshold && !picked && !positioned)  
    {
      control_gripper(true);

      trajectory.request.file = ros::package::getPath("mindwave_execute_trajectory") + 
                                "/config/" + trajFiles[0];
      ROS_INFO("Executing trajectory %s", trajFiles[0].c_str());

      if (!traj_client.call(trajectory))
      { 
          ROS_ERROR ("Failed to execute [%s] trajectory", trajFiles[0].c_str());
          //break;
      }
      usleep(500000);
      
      positioned = true;
      picked = false;

      ROS_INFO("Current State %s", trajectory.response.state.c_str());

    } 

    // if the gol does not finish yet

    if(msg.meditation >= m_threshold && !picked && positioned)
    {
      // pick object up
      control_gripper(false);
      picked = true;
      positioned = false;
    }

    //ROS_INFO("DEBUG Current State %s", trajectory.response.state.c_str());
    
    if(msg.attention >= a_threshold && picked && !positioned)  
    {

      trajectory.request.file = ros::package::getPath("mindwave_execute_trajectory") + 
                                "/config/" + trajFiles[1];
      ROS_INFO("Executing trajectory %s", trajFiles[1].c_str());
      
      if (!traj_client.call(trajectory))
      { 
          ROS_ERROR ("Failed to execute [%s] trajectory", trajFiles[1].c_str());
          //break;
      }

      usleep(500000);

      positioned = true;
      picked = true;

    } 

    if (msg.meditation >= m_threshold && picked && positioned)
    {
      control_gripper(true);
      picked = false;
      positioned = false;
    }

  } 

  /*
    This function publishes a ros message for manipulation of the robotiq hand

    Args:
      open : if it is true then open the hand, if not then close 
  */
  void control_gripper(bool open)
  {
        
    if(open){
      // publish the rostopic to open hand
      //rostopic pub --once left_hand/command robotiq_s_model_articulated_msgs/SModelRobotOutput {1,0,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0}
      cmd.rACT = 1;
      cmd.rMOD = 0;
      cmd.rGTO = 1;
      cmd.rATR = 0;
      cmd.rICF = 0;
      cmd.rICS = 0;
      cmd.rPRA = 0; // open, close
      cmd.rSPA = 255; // speed 22~110
      cmd.rFRA = 150; // force 15~60
      cmd.rPRB = 155; 
      cmd.rSPB = 0;
      cmd.rFRB = 0;
      cmd.rPRC = 255;
      cmd.rSPC = 0;
      cmd.rFRC = 0;
      cmd.rPRS = 0;
      cmd.rSPS = 0;
      cmd.rFRS = 0;
   

    }else // close hand
    {
      cmd.rACT = 1;
      cmd.rMOD = 0;
      cmd.rGTO = 1;
      cmd.rATR = 0;
      cmd.rICF = 0;
      cmd.rICS = 0;
      cmd.rPRA = 150;
      cmd.rSPA = 255;
      cmd.rFRA = 255;
      cmd.rPRB = 155;
      cmd.rSPB = 0;
      cmd.rFRB = 0;
      cmd.rPRC = 255;
      cmd.rSPC = 0;
      cmd.rFRC = 0;
      cmd.rPRS = 0;
      cmd.rSPS = 0;
      cmd.rFRS = 0;
    }

    pub.publish(cmd);
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reproduce_trajectory_node");

  ArmTeleop arm;
  
  ros::spin();
  
  return(0);
}
