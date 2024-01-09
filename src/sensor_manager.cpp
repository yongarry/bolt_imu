#include "sensor_manager.h"
#include <signal.h>

void GuiCommandCallback(const std_msgs::StringConstPtr &msg)
{
    if (msg->data == "imureset")
    {
        imu_reset_signal_ = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_manager");
    ros::NodeHandle nh_;
    ros::Publisher imu_pub;
    ros::Subscriber gui_command_sub_;
    mscl::Connection con_;
    bool imu_ok = true;
    imu_pub = nh_.advertise<sensor_msgs::Imu>("/bolt/imu", 100);
    gui_command_sub_ = nh_.subscribe("/bolt/command", 100, GuiCommandCallback);

    try
    {
        con_ = mscl::Connection::Serial("/dev/ttyACM0", 115200);
    }
    catch (mscl::Error &err)
    {
        std::cout << "imu connection error " << err.what() << std::endl;
        imu_ok = false;
    }
    if (imu_ok)
    {
        mscl::InertialNode node(con_);
        sensor_msgs::Imu imu_msg;

        MX5IMU mx5(node);

        mx5.initIMU();

        int cycle_count = 0;

        auto t_begin = std::chrono::steady_clock::now();

        while (ros::ok())
        {
            ros::spinOnce();

            cycle_count++;

            // if signal_ imu reset

            if (imu_reset_signal_)
            {
                mx5.resetEFIMU();
                imu_reset_signal_ = false;
            }

            int imu_state__;
            imu_msg = mx5.getIMU(imu_state__);

            static int no_imu_count = 0;
            static int yes_imu_count = 0;
            if (imu_state__ == -1)
            {
                no_imu_count++;
            }
            else
            {
                yes_imu_count++;
                mx5.checkIMUData();

                imu_pub.publish(imu_msg);
            }

            if ((cycle_count % 1000) == 0)
            {
                static int tb_;
                int ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t_begin).count();

                if ((mx5.packet_num < 490) || (mx5.packet_num > 510))
                {
                    std::cout << (ts - tb_) << " : imu packet error, count : " << mx5.packet_num << std::endl;
                }
                tb_ = ts;
                mx5.packet_num = 0;
            }
        }
        std::cout << "imu end" << std::endl;
        mx5.endIMU();
    }

    return 0;
}

