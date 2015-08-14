#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

#include <mindwave_execute_trajectory/ExecTraj.h>
#include <mindwave_msgs/Mindwave.h>

class ArmTeleop
{
  private:
 
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::ServiceClient traj_client;
  mindwave_execute_trajectory::ExecTraj trajectory;
    
  int m_threshold;
  int a_threshold;

  const std::string trajFiles[2]={"init_pose.csv"};

  public: 
  ArmTeleop()
  { 
    
    ros::NodeHandle n("~");

    sub = nh.subscribe<mindwave_msgs::Mindwave>("/mindwave", 1, &ArmTeleop::mindwaveCallback, this);
    //pub = nh.advertise<robotiq_s_model_articulated_msgs::SModelRobotOutput>("left_hand/command", 10);
    // Start the service
    ros::NodeHandle nh;

    traj_client = nh.serviceClient<mindwave_execute_trajectory::ExecTraj>("/execute_trajectory");

    //trajectory.request.file = ros::package::getPath("mindwave_execute_trajectory") + 
                                "/config/" + trajFiles[0];

    //mindwave_execute_trajectory::ExecTraj trajectory;

    ROS_INFO("Executing a predefined trajectory %s", trajFiles[0].c_str());

    n.param<int>("meditation_threshold", m_threshold, 50);
    n.param<int>("attention_threshold", a_threshold, 80);

  }
  
  ~ArmTeleop()   { }
  
  void mindwaveCallback(const mindwave_msgs::Mindwave msg){
   
    if(msg.attention > a_threshold)  
    {

      if (!traj_client.call(trajectory))
      { 
          trajectory.request.file = ros::package::getPath("mindwave_execute_trajectory") + 
                                "/config/" + trajFiles[0];
          ROS_ERROR ("Failed to execute [%s] trajectory", trajFiles[0].c_str());
          //break;
      }
 
    } if (msg.meditation > m_threshold)
    {
      //control_gripper()
    }
     usleep(100000);
  }

  void control_gripper(bool open)
  {
    if(open){
      //rostopic pub --once left_hand/command robotiq_s_model_articulated_msgs/SModelRobotOutput {1,0,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0}


    }else // close hand
    {


    }
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reproduce_trajectory_node");

  ArmTeleop arm;
  
  ros::spin();
  
  return(0);
}
