/*
*  Copyright 2024 (C) Richard Kroesen <RichardKroesen>, Jeroen Veen <ducroq> & Victor Hogeweij <Hoog-V>
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
* This file is part of the Face_Detector.Cpp library
*
* Author:         Jeroen Veen <ducroq>
*
*/
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        declare_and_get_parameters();
        initialize_camera();
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / camera_fps_)),
            std::bind(&CameraNode::timer_callback, this));
    }

private:
    void declare_and_get_parameters()
    {
        declare_parameter("camera_index", 0);
        declare_parameter("camera_fps", 30);
        declare_parameter("camera_width", 640);
        declare_parameter("camera_height", 480);

        camera_index_ = get_parameter("camera_index").as_int();
        camera_fps_ = get_parameter("camera_fps").as_int();
        camera_width_ = get_parameter("camera_width").as_int();
        camera_height_ = get_parameter("camera_height").as_int();
    }

    void initialize_camera()
    {
        cam_ = cv::VideoCapture(camera_index_, cv::CAP_V4L2);
        if (!cam_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Cannot open camera!");
            rclcpp::shutdown();
            return;
        }
        cam_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
        cam_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
        cam_.set(cv::CAP_PROP_FPS, camera_fps_);
    }

    void timer_callback()
    {
        cv::Mat frame;
        if (cam_.read(frame))
        {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_->publish(*msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame, retrying...");
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cam_;
    int camera_index_;
    int camera_fps_;
    int camera_width_;
    int camera_height_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}