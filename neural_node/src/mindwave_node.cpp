#include <ros/ros.h>

class MindWave
{
  private:
    /** Node handle */
    ros::NodeHandle nh;
    /** Subscriber */
    ros::Subscriber sub;
    ros::Publisher pub;

    
  public:
    /**
      Constructor
    */
    MindWave(){
      pub = nh.advertise<sensor_msgs>("neural", 1);
    }

    void readPackage(){

    }

    int main(int argc, char **argv){

      ros::init(argc, argv, "mindwave");
      ros::NodeHandle nh;
       
      ros::NodeHandle pnh("~");

      ros::Rate r(30);
      
      while (ros::ok()) {
        ros::spinOnce();
        readPackage();
    		r.sleep();
      }
      
      return 0;
    }
}; 