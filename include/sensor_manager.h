#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "mx5_imu.h"

bool imu_reset_signal_ = true;
int packet_num;

void GuiCommandCallback(const std_msgs::StringConstPtr &msg);
