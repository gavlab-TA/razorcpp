#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <razorcpp/razor_interface.h>

class RazorNode
{
    private:
        ros::NodeHandle n;
        ros::Publisher imu_pub;
        ros::Publisher mag_pub;
        ros::Publisher temp_pub;

        sensor_msgs::Imu imu_msg;
        sensor_msgs::MagneticField mag_msg;
        sensor_msgs::Temperature temp_msg;

        RazorInterface* interface;

        RazorInterface::ImuData data;

        void runPublishers()
        {
            imu_msg.angular_velocity.x = data.gyro.x;
            imu_msg.angular_velocity.y = data.gyro.y;
            imu_msg.angular_velocity.z = data.gyro.z;
            imu_msg.linear_acceleration.x = data.accel.x;
            imu_msg.linear_acceleration.y = data.accel.y;
            imu_msg.linear_acceleration.z = data.accel.z;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "razor_imu";

            mag_msg.magnetic_field.x = data.mag.x;
            mag_msg.magnetic_field.y = data.mag.y;
            mag_msg.magnetic_field.z = data.mag.z;
            mag_msg.header.stamp = ros::Time::now();
            mag_msg.header.frame_id = "razor_mag";

            temp_msg.temperature = data.temp;
            temp_msg.header.stamp = ros::Time::now();
            temp_msg.header.frame_id = "razor_temp";

            imu_pub.publish(imu_msg);
            mag_pub.publish(mag_msg);
            temp_pub.publish(temp_msg);
        }

    public:
        RazorNode() : n("/test")
        {
            std::string imu_topic;
            std::string mag_topic;
            std::string temp_topic;
            std::string port_name;
            int baudrate;

            n.param<std::string>("imu_topic", imu_topic, "/razor/imu");
            n.param<std::string>("mag_topic", mag_topic, "/razor/mag");
            n.param<std::string>("temp_topic", temp_topic, "/razor/temperature");
            n.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
            n.param<int>("baudrate", baudrate, 115200);

            imu_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 1);
            mag_pub = n.advertise<sensor_msgs::MagneticField>(mag_topic, 1);
            temp_pub = n.advertise<sensor_msgs::Temperature>(temp_topic, 1);

            interface = new RazorInterface(port_name, baudrate);
        }
        ~RazorNode()
        {
            delete interface;
        }
        void readMessage()
        {
            if(interface->readMessage(data))
            {
                runPublishers();
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "razor_node");
    RazorNode imu;
    ros::Rate loop_rate(100000);
    while (ros::ok())
    {
        imu.readMessage();
        loop_rate.sleep();
        ros::spinOnce();
    }    
    return 0;
}