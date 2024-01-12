#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "mx5_imu.h"
#include "shm_msgs.h"

class SensorManager
{
public:
    SensorManager();
    ~SensorManager() {}
    
    SHMmsgs *shm_;


    ros::NodeHandle nh_;

    ros::Publisher imu_pub;
    ros::Subscriber gui_command_sub_;

    void GuiCommandCallback(const std_msgs::StringConstPtr &msg);

    bool imu_reset_signal_ = false;

    // int packet_num;
};