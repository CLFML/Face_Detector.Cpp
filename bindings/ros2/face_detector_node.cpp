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
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <face_detection.hpp>

class FaceDetectorNode : public rclcpp::Node
{
public:
    FaceDetectorNode() : Node("face_detector_node")
    {
        declare_and_get_parameters();
        det_.load_model(CLFML_FACE_DETECTOR_CPU_MODEL_PATH);
        setup_communication();
        detection_enabled_ = true;
    }

private:
    void declare_and_get_parameters()
    {
        declare_parameter("camera_topic", "/image_raw");
        declare_parameter("face_detected_topic", "/face_detected");
        declare_parameter("face_roi_topic", "/face_roi");
        declare_parameter("face_landmarks_topic", "/face_landmarks");

        camera_topic_ = get_parameter("camera_topic").as_string();
        face_detected_topic_ = get_parameter("face_detected_topic").as_string();
        face_roi_topic_ = get_parameter("face_roi_topic").as_string();
        face_landmarks_topic_ = get_parameter("face_landmarks_topic").as_string();
    }

    void setup_communication()
    {
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, rclcpp::SensorDataQoS(), 
            std::bind(&FaceDetectorNode::imageCb, this, std::placeholders::_1));
        
        face_detected_pub_ = create_publisher<std_msgs::msg::Int32>(face_detected_topic_, 10);
        face_roi_pub_ = create_publisher<sensor_msgs::msg::RegionOfInterest>(face_roi_topic_, 10);        
        face_landmarks_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(face_landmarks_topic_, 10);

        toggle_service_ = create_service<std_srvs::srv::SetBool>(
            "toggle_face_detection", 
            std::bind(&FaceDetectorNode::toggleDetection, this, std::placeholders::_1, std::placeholders::_2));
    }

    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!detection_enabled_) return;
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        det_.load_image(cv_ptr->image);

        publish_face_detected();
        publish_face_roi();
        publish_face_landmarks();
    }

    void publish_face_detected()
    {
        std_msgs::msg::Int32 face_detected_msg = std_msgs::msg::Int32();
        face_detected_msg.data = det_.detected() + 1; // +1 because detector returns -1 for no face and 0 for face detected!
        face_detected_pub_->publish(face_detected_msg);

        if (face_detected_msg.data)
            RCLCPP_INFO(get_logger(), "face detected");
    }

    void publish_face_roi()
    {
        cv::Rect face_roi = det_.get_face_roi();
        sensor_msgs::msg::RegionOfInterest roi_msg = sensor_msgs::msg::RegionOfInterest();
        roi_msg.x_offset = face_roi.x;
        roi_msg.y_offset = face_roi.y;
        roi_msg.width = face_roi.width;
        roi_msg.height = face_roi.height;
        face_roi_pub_->publish(roi_msg);
    }    

    void publish_face_landmarks()
    {
        std::array<cv::Point, 6> face_keypoints = det_.get_face_landmarks();
        for (const cv::Point& keypoint : face_keypoints)
        {
            geometry_msgs::msg::PointStamped landmark_msg = geometry_msgs::msg::PointStamped();
            landmark_msg.header.stamp = now();
            landmark_msg.header.frame_id = "camera_frame";
            landmark_msg.point.x = keypoint.x;
            landmark_msg.point.y = keypoint.y;
            landmark_msg.point.z = 0.0;  // Assuming 2D points
            face_landmarks_pub_->publish(landmark_msg);
        }
    }

    void toggleDetection(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        detection_enabled_ = request->data;
        response->success = true;
        response->message = detection_enabled_ ? "Face detection enabled" : "Face detection disabled";
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr face_detected_pub_;
    rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr face_roi_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr face_landmarks_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_service_;

    CLFML::FaceDetection::FaceDetector det_;
    std::string camera_topic_;
    std::string face_detected_topic_;
    std::string face_roi_topic_;
    std::string face_landmarks_topic_;
    bool detection_enabled_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceDetectorNode>());
    rclcpp::shutdown();
    return 0;
}