#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>

#define RAD2DEG(x) ((x)*180./M_PI)

using std::placeholders::_1;

int count;
int lidar_Array[180] = {0,};
int last_lidar_Array[180] = {0,};

class ReadingLaser : public rclcpp::Node
{
    public:
        ReadingLaser() : Node("reading_laser")
        {
            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            Subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "laser_scan",
                default_qos,
                std::bind(&ReadingLaser::topic_callback, this, _1)
            );
        }
    private:
        void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
            count = (int)(360./RAD2DEG(_msg->angle_increment));

            for(int i = 0; i < count; i++)
            {
                int degree = RAD2DEG(_msg->angle_min + _msg->angle_increment * i) + 180;
                int range = _msg->ranges[i]*100;
                if( (degree<=270)&&(degree>=90) )
                {
                    lidar_Array[degree-91] = range;
                    if(range == 0)
                    {
                        lidar_Array[degree-91] = last_lidar_Array[degree-91];
                    }
                    else
                    {
                        lidar_Array[degree-91] = range;
                    }
                    last_lidar_Array[degree-91] = lidar_Array[degree-91];
                }

            }
            RCLCPP_INFO(this->get_logger(), "I heard: '%d' '%d' '%d'", lidar_Array[0], lidar_Array[89], lidar_Array[179]);
        }
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Subscription_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReadingLaser>();
    RCLCPP_INFO(node->get_logger(), "Hello my friengs");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
