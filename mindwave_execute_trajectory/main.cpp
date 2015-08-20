#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <std_srvs/Empty.h>
#include "mindwave_execute_trajectory/ExecTraj.h"


using namespace std;

typedef  vector< vector <double> > Trajectory;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

static const std::string FOLLOW_JOINT_NAME = "/arm_controller/follow_joint_trajectory";

/*
  This class allows to publish a ROS service to moving an Arm 
  using the Joint Trajectory Action interface with a set of waypoints.

*/
class Trajectories
{

public:
  
    TrajClient* traj_client;

    Trajectory trajectory;

    Trajectories()
    {
      // creates the action client
      // true causes the client to spin its own thread
      traj_client = new TrajClient(FOLLOW_JOINT_NAME, true);

      // wait for the action server to start
      traj_client->waitForServer(); //will wait for infinite time
    }

    ~Trajectories()
    {
        delete traj_client;
    }

    /*
      This method executes a trajectory in a .csv file
    */
    std::string exec(std::string file)
    {
      ROS_INFO("Action server started, sending goal.");
      trajectory = readFile (file);

      if (trajectory.size() == 0) return "trajectory file not found";

      // send a goal to the action;
      traj_client->sendGoal( buildGoal (trajectory) );

      //wait for the action to return
      bool finished_before_timeout = traj_client->waitForResult(ros::Duration(15.0));
      
      actionlib::SimpleClientGoalState current_state = traj_client->getState();

      if (finished_before_timeout)
      {
        ROS_INFO("Action finished: %s", current_state.toString().c_str());
      }
      else
      {
        ROS_INFO("Action did not finish before the time out.");
        //traj_client.cancelGoal();
        return "timeout";
      }

      return "ok";
    }
    
    /*
      This function prints the Trajectory matrix of points
    */
    void printTraj(Trajectory trajectory)
    {
      ROS_INFO("Printing list of points of the trajectory.");

      cout<<"tam traj "<<trajectory.size()<<endl;
      
      for(int i=0 ; i < trajectory.size(); i++)
      {

        std::vector<double> point = trajectory.at(i);
        cout<<"tam "<<point.size()<<endl;
        for(int x=0 ; x < point.size(); x++){
         
          cout<<point.at(x)<<",";
        }
        cout<<endl;
      }

    }

    Trajectory getTrajectory()
    {
      return trajectory;
    }

    /*
      This function reads a .csv file to load a vector matrix 
    */
    Trajectory readFile (std::string fileName)
    {
      Trajectory listOfPoints;
      string line;
      ifstream myfile (fileName.c_str());
      if (myfile.is_open())
      {
        while ( getline (myfile,line) )
        {
          std::string delimiter = ",";

          size_t pos = 0;
          std::string token;
          vector<double> point;
          
          while ((pos = line.find(delimiter)) != std::string::npos)
          {
            token = (line.substr(0, pos));
            
            point.push_back (strtod(token.c_str(),NULL));
            line.erase(0, pos + delimiter.length());
            
          }

          // last token
          if(line.length()>0)
            point.push_back (strtod(line.c_str(),NULL));

          listOfPoints.push_back(point);
        }
        myfile.close();
      }
      else
        cout<< "Unable to open file [%s]"<< fileName.c_str();

      return listOfPoints;    
    }

    /* 
      Return the current state of the goal action
    */
    std::string getState()
    {
      return traj_client->getState().toString();
    }

 private:
    /*
      This function builds the trajectory to follow the endeffector 
      according to Universal Robot joints to execution of a matrix of points.
    */
    control_msgs::FollowJointTrajectoryGoal buildGoal ( Trajectory trajectory)
    {
        
        control_msgs::FollowJointTrajectoryGoal goal;

        goal.trajectory.header.stamp = ros::Time::now()  + ros::Duration(1.0);
        goal.trajectory.header.frame_id = "base_link";

        // First, the joint names, which apply to all waypoints for Universal Robots
        goal.trajectory.joint_names.push_back("shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("elbow_joint");
        goal.trajectory.joint_names.push_back("wrist_1_joint");
        goal.trajectory.joint_names.push_back("wrist_2_joint");
        goal.trajectory.joint_names.push_back("wrist_3_joint");

        // setting the number of points in the trajectory
        goal.trajectory.points.resize( trajectory.size());

        // First trajectory point
        // Positions
        for (int i=0 ; i < trajectory.size(); i++)
        {

            std::vector<double> point = trajectory.at(i);

            goal.trajectory.points[i].positions.resize(6);
            goal.trajectory.points[i].velocities.resize(6);
            goal.trajectory.points[i].accelerations.resize(6);

            goal.trajectory.points[i].positions[0] = point.at(0);
            goal.trajectory.points[i].velocities[0] = 0.0;
         
            goal.trajectory.points[i].positions[1] = point.at(1);
            goal.trajectory.points[i].velocities[1] = 0.0;
          
            goal.trajectory.points[i].positions[2] = point.at(2);
            goal.trajectory.points[i].velocities[2] = 0.0;
            
            goal.trajectory.points[i].positions[3] = point.at(3);
            goal.trajectory.points[i].velocities[3] = 0.0;
          
            goal.trajectory.points[i].positions[4] = point.at(4);
            goal.trajectory.points[i].velocities[4] = 0.0;
           
            goal.trajectory.points[i].positions[5] = point.at(5);
            goal.trajectory.points[i].velocities[5] = 0.0;
            
            //goal.trajectory.points[i].positions[6] = point.at(6);
            //goal.trajectory.points[i].velocities[6] = point.at(13);

            // To be reached 1 second after starting along the trajectory
            goal.trajectory.points[i].time_from_start = ros::Duration(1.0 + i * 0.02);
        }

        return goal;
    }

};

/*
  Request callback function to calling the service node
*/
bool trajectory_execution_callback(mindwave_execute_trajectory::ExecTraj::Request &req,
                                   mindwave_execute_trajectory::ExecTraj::Response &res)
{
    Trajectories traj;
    
    if (traj.exec(req.file) == "ok")
    {
       res.state = traj.getState();
      return true;
    }else
       res.state = traj.getState();
       
    return false;
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "execute_trajectory");

  ROS_INFO("Waiting for action server to start.");

  if (argc > 1)
  {
    Trajectories traj;
    std::string file(argv[1]);
    traj.exec(file);
  }


  // Start the service
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("execute_trajectory", &trajectory_execution_callback);

  ROS_INFO("Ready execute trajectories");
  ros::spin();

  return 0;
}
