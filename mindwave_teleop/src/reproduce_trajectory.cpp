#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

#include <robotiq_s_model_articulated_msgs/SModelRobotInput.h>
#include <robotiq_s_model_articulated_msgs/SModelRobotOutput.h>
//#include <robotiq_msgs/SModelRobotInput.h>
//#include <robotiq_msgs/SModelRobotOutput.h>


#include <mindwave_msgs/Mindwave.h>
#include <mindwave_execute_trajectory/ExecTraj.h>

/*
 This node class executes a predefined trajectory and
 interoperate with Mindwave message.
*/

class ArmTeleop
{
  private:
 
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::ServiceClient traj_client;
  mindwave_execute_trajectory::ExecTraj trajectory;

  robotiq_s_model_articulated_msgs::SModelRobotOutput cmd; 

  int m_threshold;
  int a_threshold;
  bool picked;
  bool positioned;


  const std::string trajFiles[2]={"pick2.csv", "place2.csv"};

  public: 
  ArmTeleop()
  { 
    
    ros::NodeHandle n("~");

    n.param<int>("meditation_threshold", m_threshold, 80);
    n.param<int>("attention_threshold", a_threshold, 50);

    picked = false;
    positioned = false;

    sub = nh.subscribe<mindwave_msgs::Mindwave>("/mindwave", 1, &ArmTeleop::mindwaveCallback, this);
    pub = nh.advertise<robotiq_s_model_articulated_msgs::SModelRobotOutput>("left_hand/command", 10);
    
    // Start the service
    ros::NodeHandle nh;

    traj_client = nh.serviceClient<mindwave_execute_trajectory::ExecTraj>("/execute_trajectory");

    //trajectory.request.file = ros::package::getPath("mindwave_execute_trajectory") + 
                                "/config/" + trajFiles[0];

    //mindwave_execute_trajectory::ExecTraj trajectory;

    ROS_INFO("Executing a predefined trajectory %s", trajFiles[0].c_str()); 

    control_gripper(true);

  }
  
  ~ArmTeleop()   { }
  
  void mindwaveCallback(const mindwave_msgs::Mindwave msg){
    

    if(msg.attention > a_threshold && !picked && !positioned)  
    {
      
      trajectory.request.file = ros::package::getPath("mindwave_execute_trajectory") + 
                                "/config/" + trajFiles[0];

      if (!traj_client.call(trajectory))
      { 
          ROS_ERROR ("Failed to execute [%s] trajectory", trajFiles[0].c_str());
          //break;
      }

      usleep(100000);
      positioned = true;

    } if(msg.meditation > m_threshold && !picked && positioned)
    {
      // close that hand
      control_gripper(false);
      picked = true;
    }
    
    if(msg.attention > a_threshold && picked && positioned)  
    {

      trajectory.request.file = ros::package::getPath("mindwave_execute_trajectory") + 
                                "/config/" + trajFiles[1];

      if (!traj_client.call(trajectory))
      { 
          ROS_ERROR ("Failed to execute [%s] trajectory", trajFiles[1].c_str());
          //break;
      }

      usleep(100000);

      positioned = false;

    } if (msg.meditation > m_threshold && picked && !positioned)
    {
      control_gripper(true);
      picked = false;
    }
     

  } 

  void control_gripper(bool open)
  {
        
    if(open){
      // publish the rostopic to open hand
      //rostopic pub --once left_hand/command robotiq_s_model_articulated_msgs/SModelRobotOutput {1,0,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0}
      cmd.rACT = 1;
      cmd.rMOD = 1;
      cmd.rGTO = 1;
      cmd.rATR = 0;
      cmd.rICF = 0;
      cmd.rICS = 0;
      cmd.rPRA = 0; // open, close
      cmd.rSPA = 255; // speed 22~110
      cmd.rFRA = 255; // force 15~60
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
      cmd.rFRA = 0;
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
