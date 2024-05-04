#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        this->declare_parameter<double>("window_size", 3.0);
        this->declare_parameter<double>("max_dist", 7.0);
        // this->declare_parameter<double>("bub_radius", 20.0);
        this->declare_parameter<double>("disparity_thres", 0.3);
        this->declare_parameter<double>("safe_dist", 3.0);
        this->declare_parameter<double>("width_car", 0.34);
        this->declare_parameter<double>("high_speed", 0.5);
        this->declare_parameter<double>("mid_speed", 0.15);
        this->declare_parameter<double>("low_speed", 0.1);
        this->declare_parameter<double>("move_angle", 70.0);
        this->declare_parameter<double>("range_frac", 0.48);
        // this->declare_parameter<double>("kd", 0.0001);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    const double PI = 3.14159265358979323846;
    void preprocess_lidar(std::vector<double>& ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        int window_size = static_cast<int>(this->get_parameter("window_size").as_double());
        double max_dist = this->get_parameter("max_dist").as_double();
        RCLCPP_INFO(this->get_logger(), "max_dist: %f", max_dist);
        RCLCPP_INFO(this->get_logger(), "window_size: %d", window_size);
        std::vector<double> temp_ranges = ranges;
        // RCLCPP_INFO(this->get_logger(), "Temp range -90 before: %f", temp_ranges[720]);
        for (int i = 0; i < static_cast<int>(ranges.size()-window_size); i++) {
            // // Calculate the end of the current window, ensuring it doesn't go beyond the vector's end
            // double windowEnd = std::min(static_cast<double>(ranges.size()), i + window_size);

            // // Calculate the sum and then the mean of the current window
            // double windowSum = std::accumulate(ranges.begin() + i, ranges.begin() + windowEnd, 0.0);
            // double windowMean = windowSum / (windowEnd - i);
            // RCLCPP_INFO(this->get_logger(), "ranges[%d]: %f", i, ranges[i]);
            double sum = 0.0;
            for(int j=0; j<=window_size; j++){
                sum += ranges[i+j];
            }
            // temp_ranges[i] = std::min((sum / (window_size + 1)), max_dist);
            temp_ranges[i] = std::min(ranges[i], max_dist);
            
        }
        // RCLCPP_INFO(this->get_logger(), "Temp range -90: %f", temp_ranges[720]);
        ranges = temp_ranges;
        return;
    }

    size_t dynamic_bub_radius(std::vector<double>& ranges, int index, double angle_increment){
        double width = this->get_parameter("width_car").as_double() / 2;
        double angle_range = width / ranges[index];
        size_t bub_radius = static_cast<size_t>(angle_range / angle_increment);
        return bub_radius;
    }

    void disparitry_extender(std::vector<double>& ranges){
        // size_t bub_radius = static_cast<size_t>(this->get_parameter("bub_radius").as_double());
        size_t bub_radius;
        double disparity_thres = this->get_parameter("disparity_thres").as_double();
        RCLCPP_INFO(this->get_logger(), "bub_radius: %d", bub_radius);
        RCLCPP_INFO(this->get_logger(), "disparity_thres: %f", disparity_thres);

        for (size_t i = 0; i < ranges.size() - 1; ++i) {
        // Check if the disparity between the current and next element exceeds the threshold
            if (std::abs(ranges[i] - ranges[i + 1]) >= disparity_thres) {
                // Determine the direction of extension based on which value is smaller
                RCLCPP_INFO(this->get_logger(), "Dis Extend Working with i %d and i+1 %d, %f and %f with dist %f", i, i+1, ranges[i], ranges[i+1], ranges[i]-ranges[i+1]);
                if (ranges[i] < ranges[i + 1]) {
                    // Extend forward from the current element
                    size_t bub_radius = dynamic_bub_radius(ranges, i, 0.004351851996034384);
                    size_t end = std::min((i + 1 + bub_radius), ranges.size()); // Ensure we don't underflow
                    // std::fill(ranges.begin() + i, ranges.begin() + end, ranges[i]);
                    std::fill(ranges.begin() + i + 1, ranges.begin() + end, ranges[i]);
                    i = end + 1;
                    RCLCPP_INFO(this->get_logger(), "Zeroed Value with index %d: %f", i+1, ranges[i+1]);
                } 
                else {
                    // Extend backwards from the next element
                    size_t bub_radius = dynamic_bub_radius(ranges, i+1, 0.004351851996034384);
                    size_t start = (i >= bub_radius) ? i - bub_radius : 0;
                    // std::fill(ranges.begin() + start, ranges.begin() + i + 2, ranges[i + 1]);
                    std::fill(ranges.begin() + start, ranges.begin() + i + 1, ranges[i+1]);
                    RCLCPP_INFO(this->get_logger(), "Zeroed Value with index %d: %f", start+1, ranges[start+1]);
                    // i = (i >= bub_radius) ? i - bub_radius : 0;
                }
            }
        }
    }

    std::pair<int, int> find_max_gap(std::vector<double>& ranges)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        double safe_dist = this->get_parameter("safe_dist").as_double();
        RCLCPP_INFO(this->get_logger(), "safe_dist: %f", safe_dist);
        int start = 0;
        int end = 0;
        int max_len = 0;
        int cur_len = 0;

        int len = ranges.size();
        for (int i = 0; i < len; ++i) {
            if (ranges[i] >= safe_dist) {
                cur_len++;
                if (cur_len > max_len) {
                    max_len = cur_len;
                    end = i; // end is now the actual last index in the gap
                }
            } else {
                cur_len = 0;
            }
        }

        if (max_len > 0) {
            start = end - max_len + 1; // Adjust start index based on max_len
            return std::make_pair(start, end); // Make end exclusive again if desired
        } else {
            return std::make_pair(0, 0); // Indicate no gap found
        }
        // int start = 0;
        // int end = 0;
        // int max_len = 0;
        // int cur_len = 0;

        // int len = ranges.size();
        // for(int i=0; i<len; ++i){
        //     if(ranges[i] >= safe_dist){
        //         cur_len += 1;
        //         if(cur_len > max_len){
        //             max_len = cur_len;
        //             end = i;
        //             start = end - max_len + 1;
        //         }
        //     }
        //     else{
        //         cur_len = 0;
        //     }
        // }
        // return std::make_pair(start, end);
    }

    int find_best_point(int start, int end)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        double range_frac = this->get_parameter("range_frac").as_double();
        int result = static_cast<int>(static_cast<double>(start) + static_cast<double>((end - start)*range_frac));
        return result;
    }

    int find_furthest_point(int start, int end, std::vector<double>& ranges){
        double furthest_dist = 0;
        int best_idx = start;
        for (size_t i=start; i<=end; i++){
            if (ranges[i] > furthest_dist){
                furthest_dist = ranges[i];
                best_idx = i;
            }
        }
        return best_idx;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message
        double move_angle = this->get_parameter("move_angle").as_double();
        double left_angle_deg = move_angle;
        // double right_angle_deg = -90;
        // double right_angle_rad = right_angle_deg * PI / 180;
        double left_angle_rad = left_angle_deg * PI / 180;

        // int start_idx = static_cast<int>(((scan_msg->angle_max - left_angle_rad) / (scan_msg->angle_max - scan_msg->angle_min)) * 1080);
        // int end_idx = 1080 - start_idx;
        int start_idx = 540 - static_cast<int>(left_angle_rad / scan_msg->angle_increment);
        int end_idx = 540 + static_cast<int>(left_angle_rad / scan_msg->angle_increment);
        // RCLCPP_INFO(this->get_logger(), "Start Index: %d", start_idx);
        // RCLCPP_INFO(this->get_logger(), "End Index: %d", end_idx);
        std::vector<double> ranges;
        ranges.assign(scan_msg->ranges.begin() + start_idx, scan_msg->ranges.begin() + end_idx);

        preprocess_lidar(ranges);
        RCLCPP_INFO(this->get_logger(), "Angle 0 after preprocessing: %f", ranges[360]);
        RCLCPP_INFO(this->get_logger(), "Angle 90 after preprocessing: %f", ranges[718]);
        RCLCPP_INFO(this->get_logger(), "Angle -90 after preprocessing: %f", ranges[2]);
        disparitry_extender(ranges);
        RCLCPP_INFO(this->get_logger(), "Angle 0 after dis extend: %f", ranges[360]);
        RCLCPP_INFO(this->get_logger(), "Angle 90 after dis extend: %f", ranges[718]);
        RCLCPP_INFO(this->get_logger(), "Angle -90 after dis extend: %f", ranges[2]);
        RCLCPP_INFO(this->get_logger(), "Range Size: %d", ranges.size());
        std::pair<int, int> indexes = find_max_gap(ranges);
        int right = indexes.first + start_idx;
        int left = indexes.second + start_idx;
        RCLCPP_INFO(this->get_logger(), "Left index: %d with distance %f, Right index: %d with distance %f", left, ranges[indexes.second], right, ranges[indexes.first]);
        int best_index = find_best_point(left, right);
        // int best_index = find_furthest_point(indexes.first, indexes.second, ranges) + start_idx;
        RCLCPP_INFO(this->get_logger(), "Best index: %d with dist %f", best_index, ranges[best_index-start_idx]);
        RCLCPP_INFO(this->get_logger(), "Best index: %d with scan msg dist %f", best_index, scan_msg->ranges[best_index]);
        double steering_angle_rad = ((best_index - 540) * (scan_msg->angle_max - scan_msg->angle_min) / 1080);
        double steering_angle_deg = steering_angle_rad * 180 / PI;
        RCLCPP_INFO(this->get_logger(), "Best steering angle: %f", steering_angle_deg);
        double speed = 0;
        double high_speed = this->get_parameter("high_speed").as_double();
        double mid_speed = this->get_parameter("mid_speed").as_double();
        double low_speed = this->get_parameter("low_speed").as_double();
        if(std::abs(steering_angle_deg) < 10){
            speed = high_speed;
        }
        else if (std::abs(steering_angle_deg) >= 10 && std::abs(steering_angle_deg) <= 20){
            speed = mid_speed;
        }
        else{
            speed = low_speed;
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle_rad;

        drive_publisher->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
