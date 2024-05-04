#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>



class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1));
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    const double TTC_threshold = 1.5;
    const double PI = 3.14159265358979323846;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        std::vector<double> TTC_arr = calculate_TTC(scan_msg);
        /// TODO: publish drive/brake message

        // calculate the angle range

        double angle_range_in_degree = 90;
        double angle_range_in_rad = angle_range_in_degree * 2 * PI / 360;
        int index_amount = static_cast<int>(angle_range_in_rad / scan_msg->angle_increment);
        int leftmost_idx = static_cast<int>((1080 - index_amount) / 2);
        int rightmost_idx = static_cast<int>((1080 + index_amount) / 2);

        // for(auto TTC : TTC_arr){
        //     if(TTC < TTC_threshold){
        //         auto command = ackermann_msgs::msg::AckermannDriveStamped();
        //         command.drive.speed = 0.0;
        //         // std::cout << std::endl;
        //         // RCLCPP_INFO(this->get_logger(), "Current iTTC value: %f", TTC);
        //         RCLCPP_INFO(this->get_logger(), "Current iTTC threshold: %f", TTC_threshold);
        //         drive_publisher->publish(command);
        //         return;
        //     }
        //     // else{
        //     //     std::cout << std::endl;
        //     //     // RCLCPP_INFO(this->get_logger(), "Current iTTC value: %f", TTC);
        //     // }
        // }

        for(int i = leftmost_idx; i<= rightmost_idx; i++){
            double TTC_cur = TTC_arr[i];
            // RCLCPP_INFO(this->get_logger(), "Current speed: %f", speed);
            // RCLCPP_INFO(this->get_logger(), "Current iTTC value: %f", TTC_cur);
                if(TTC_cur < TTC_threshold){
                    auto command = ackermann_msgs::msg::AckermannDriveStamped();
                    command.drive.speed = 0.0;
                    // std::cout << std::endl;
                    // RCLCPP_INFO(this->get_logger(), "Current iTTC value: %f", TTC);
                    // RCLCPP_INFO(this->get_logger(), "Current iTTC threshold: %f", TTC_threshold);
                    RCLCPP_INFO(this->get_logger(), "The brake is engaged!");
                    drive_publisher->publish(command);
                    return;
                }
        }

        // for(int i = 0; i<= 1080; i++){
        //     double TTC_cur = TTC_arr[i];
        //     RCLCPP_INFO(this->get_logger(), "Current speed: %f", speed);
        //     RCLCPP_INFO(this->get_logger(), "Current iTTC value: %f", TTC_cur);
        //         if(TTC_cur < TTC_threshold){
        //             auto command = ackermann_msgs::msg::AckermannDriveStamped();
        //             command.drive.speed = 0.0;
        //             // std::cout << std::endl;
        //             // RCLCPP_INFO(this->get_logger(), "Current iTTC value: %f", TTC);
        //             RCLCPP_INFO(this->get_logger(), "Current iTTC threshold: %f", TTC_threshold);
        //             drive_publisher->publish(command);
        //             return;
        //         }
        // }
    }

    std::vector<double> calculate_TTC(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg){
        std::vector<double> result;
        for(size_t i=0; i<scan_msg->ranges.size(); i++){
            double range = scan_msg->ranges[i];
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // RCLCPP_INFO(this->get_logger(), "Current speed: %f", speed);
            double rate = speed * std::cos(angle);
            double iTTC;
            if(rate <= 0){
                iTTC = std::numeric_limits<double>::infinity();
            }
            else{
                iTTC = range / rate;
            }
            result.push_back(iTTC);
        }
        return result;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}