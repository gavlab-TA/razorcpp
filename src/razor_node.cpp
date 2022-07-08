#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include <razorcpp/razor_interface.h>

class RazorNode : public rclcpp::Node
{
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;

        sensor_msgs::msg::Imu imu_msg;
        sensor_msgs::msg::MagneticField mag_msg;
        sensor_msgs::msg::Temperature temp_msg;

        RazorInterface* interface;

        RazorInterface::ImuData data;

        rclcpp::TimerBase::SharedPtr timer;

        void timerCallback()
        {
            if(interface->readMessage(data))
            {
                runPublishers();
            }
        }

        void runPublishers()
        {
            imu_msg.angular_velocity.x = data.gyro.x;
            imu_msg.angular_velocity.y = data.gyro.y;
            imu_msg.angular_velocity.z = data.gyro.z;
            imu_msg.linear_acceleration.x = data.accel.x;
            imu_msg.linear_acceleration.y = data.accel.y;
            imu_msg.linear_acceleration.z = data.accel.z;
            imu_msg.header.stamp = rclcpp::Clock().now();
            imu_msg.header.frame_id = "razor_imu";

            mag_msg.magnetic_field.x = data.mag.x;
            mag_msg.magnetic_field.y = data.mag.y;
            mag_msg.magnetic_field.z = data.mag.z;
            mag_msg.header.stamp = rclcpp::Clock().now();
            mag_msg.header.frame_id = "razor_mag";

            temp_msg.temperature = data.temp;
            temp_msg.header.stamp = rclcpp::Clock().now();
            temp_msg.header.frame_id = "razor_temp";

            imu_pub->publish(imu_msg);
            mag_pub->publish(mag_msg);
            temp_pub->publish(temp_msg);
        }

    public:
        RazorNode() : Node("razor_node")
        {
            this->declare_parameter<std::string>("imu_topic", "/razor/imu");
            this->declare_parameter<std::string>("mag_topic", "/razor/mag");
            this->declare_parameter<std::string>("temp_topic", "/razor/temperature");
            this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
            this->declare_parameter<int>("baudrate", 115200);
            this->declare_parameter<int>("timer_ms", 1);

            std::string imu_topic;
            std::string mag_topic;
            std::string temp_topic;
            std::string port_name;
            int baudrate;
            int timer_ms;

            this->get_parameter<std::string>("imu_topic", imu_topic);
            this->get_parameter<std::string>("mag_topic", mag_topic);
            this->get_parameter<std::string>("temp_topic", temp_topic);
            this->get_parameter<std::string>("port_name", port_name);
            this->get_parameter<int>("baudrate", baudrate);
            this->get_parameter<int>("timer_ms", timer_ms);

            this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);
            this->mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 1);
            this->temp_pub = this->create_publisher<sensor_msgs::msg::Temperature>(temp_topic, 1);
            
            this->timer = this->create_wall_timer(std::chrono::milliseconds(timer_ms), std::bind(&RazorNode::timerCallback, this));

            interface = new RazorInterface(port_name, baudrate);
        }
        ~RazorNode()
        {
            delete interface;
        }

        
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RazorNode>());
    rclcpp::shutdown();
    return 0;
}