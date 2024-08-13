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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "sensor_msgs/msg/region_of_interest.hpp"
// #include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class FaceDetectorSubscriber : public rclcpp::Node
{
public:
    FaceDetectorSubscriber() : Node("face_detector_subscriber")
    {
        // Declare parameters for topic names
        this->declare_parameter("face_detected_topic", "/face_detected");
        this->declare_parameter("face_roi_topic", "/face_roi");
        this->declare_parameter("face_landmarks_topic", "/face_landmarks");

        // Get parameter values
        std::string face_detected_topic = this->get_parameter("face_detected_topic").as_string();
        std::string face_roi_topic = this->get_parameter("face_roi_topic").as_string();
        std::string face_landmarks_topic = this->get_parameter("face_landmarks_topic").as_string();

        // Create subscriptions
        face_detected_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            face_detected_topic, 10, 
            std::bind(&FaceDetectorSubscriber::faceDetectedCallback, this, std::placeholders::_1));

        face_roi_sub_ = this->create_subscription<sensor_msgs::msg::RegionOfInterest>(
            face_roi_topic, 10, 
            std::bind(&FaceDetectorSubscriber::faceRoiCallback, this, std::placeholders::_1));

        face_landmarks_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            face_landmarks_topic, 10, 
            std::bind(&FaceDetectorSubscriber::faceLandmarksCallback, this, std::placeholders::_1));
    }

private:
    void faceDetectedCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Face detected: %d", msg->data);
    }

    void faceRoiCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Face ROI; x: %.2f, y: %.2f, width: %.2f, height: %.2f",
                    msg->x_offset, msg->y_offset, msg->width, msg->height);
    }

    void faceLandmarksCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Face Landmark; x: %.2f, y: %.2f, z: %.2f", 
                    msg->point.x, msg->point.y, msg->point.z);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr face_detected_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr face_roi_sub_;
    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr face_roi_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr face_landmarks_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceDetectorSubscriber>());
    rclcpp::shutdown();
    return 0;
}