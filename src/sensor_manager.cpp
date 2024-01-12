#include "sensor_manager.h"
#include <signal.h>

volatile bool *prog_shutdown;

void SIGINT_handler(int sig)
{
    // std::cout << " SENSOR : shutdown Signal" << std::endl;
    *prog_shutdown = true;
}

SensorManager::SensorManager()
{
    imu_pub = nh_.advertise<sensor_msgs::Imu>("/bolt/imu", 100);
    gui_command_sub_ = nh_.subscribe("/bolt/command", 100, &SensorManager::GuiCommandCallback, this);
}

void SensorManager::GuiCommandCallback(const std_msgs::StringConstPtr &msg)
{
    if (msg->data == "imureset")
    {
        imu_reset_signal_ = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_manager");

    SensorManager sm_;

    int shm_id_;
    init_shm(shm_msg_key, shm_id_, &sm_.shm_);
    prog_shutdown = &sm_.shm_->shutdown;

    mscl::Connection con_;
    bool imu_ok = true;
    try
    {
        system("sudo chmod 666 /dev/ttyACM0");
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

        sm_.shm_->imu_state = 0;

        while (!sm_.shm_->shutdown && ros::ok())
        // while (ros::ok())
        {
            ros::spinOnce();

            cycle_count++;

            // if signal_ imu reset

            if (sm_.imu_reset_signal_)
            {
                mx5.resetEFIMU();
                sm_.imu_reset_signal_ = false;
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
                sm_.shm_->imuWriting = true;
                sm_.shm_->imu_state = imu_state__;
                // std::cout<< "imu_state__: " << imu_state__<<std::endl;
                sm_.shm_->pos_virtual[0] = imu_msg.orientation.x;
                sm_.shm_->pos_virtual[1] = imu_msg.orientation.y;
                sm_.shm_->pos_virtual[2] = imu_msg.orientation.z;
                sm_.shm_->pos_virtual[3] = imu_msg.orientation.w;
                sm_.shm_->vel_virtual[0] = imu_msg.angular_velocity.x;
                sm_.shm_->vel_virtual[1] = imu_msg.angular_velocity.y;
                sm_.shm_->vel_virtual[2] = imu_msg.angular_velocity.z;
                sm_.shm_->imu_acc[0] = imu_msg.linear_acceleration.x;
                sm_.shm_->imu_acc[1] = imu_msg.linear_acceleration.y;
                sm_.shm_->imu_acc[2] = imu_msg.linear_acceleration.z;
                // std::cout<<shm_->pos_virtual[3]<<shm_->pos_virtual[4]<<shm_->pos_virtual[5]<<shm_->pos_virtual[6]<<std::endl;

                sm_.imu_pub.publish(imu_msg);
                sm_.shm_->imuWriting = false;
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
    deleteSharedMemory(shm_id_, sm_.shm_);

    return 0;
}

