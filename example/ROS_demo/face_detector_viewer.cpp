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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class FaceDetectorViewer : public rclcpp::Node
{
public:
    FaceDetectorViewer() : Node("face_detector_viewer")
    {
        // Declare parameters for topic names
        this->declare_parameter("camera_topic", "/image_raw");
        this->declare_parameter("face_detected_topic", "/face_detected");
        this->declare_parameter("face_roi_topic", "/face_roi");
        this->declare_parameter("face_landmarks_topic", "/face_landmarks");

        // Get parameter values
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        std::string face_detected_topic = this->get_parameter("face_detected_topic").as_string();
        std::string face_roi_topic = this->get_parameter("face_roi_topic").as_string();
        std::string face_landmarks_topic = this->get_parameter("face_landmarks_topic").as_string();

        // Create subscriptions
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10, 
            std::bind(&FaceDetectorViewer::imageCallback, this, std::placeholders::_1));

        face_detected_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            face_detected_topic, 10, 
            std::bind(&FaceDetectorViewer::faceDetectedCallback, this, std::placeholders::_1));

        face_roi_sub_ = this->create_subscription<sensor_msgs::msg::RegionOfInterest>(
            face_roi_topic, 10, 
            std::bind(&FaceDetectorViewer::faceRoiCallback, this, std::placeholders::_1));

        face_landmarks_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            face_landmarks_topic, 10, 
            std::bind(&FaceDetectorViewer::faceLandmarksCallback, this, std::placeholders::_1));

        // Create OpenCV window
        cv::namedWindow("Face Detection Viewer", cv::WINDOW_NORMAL);
        cv::resizeWindow("Face Detection Viewer", 640, 480);
    }

    ~FaceDetectorViewer()
    {
        cv::destroyAllWindows();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            current_frame_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            updateDisplay();
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void faceDetectedCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        face_detected_ = msg->data;
        updateDisplay();
    }

    void faceRoiCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
    {
        face_roi_ = cv::Rect(msg->x_offset, msg->y_offset, msg->width, msg->height);
        updateDisplay();
    }

    void faceLandmarksCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        cv::Point landmark(msg->point.x, msg->point.y);
        face_landmarks_.push_back(landmark);
        if (face_landmarks_.size() > 6) {
            face_landmarks_.erase(face_landmarks_.begin());
        }
        updateDisplay();
    }

    void updateDisplay()
    {
        if (current_frame_.empty()) return;

        cv::Mat display_frame = current_frame_.clone();

        // Draw face detection status
        std::string top_left_text = "Detected: " + std::to_string(face_detected_);
        cv::putText(display_frame, top_left_text, cv::Point(20, 70), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 2);

        // Draw face ROI
        if (face_roi_.width > 0 && face_roi_.height > 0) {
            cv::rectangle(display_frame, face_roi_, cv::Scalar(0, 255, 0), 2);
        }

        // Draw face landmarks
        for (const cv::Point& landmark : face_landmarks_) {
            cv::circle(display_frame, landmark, 2, cv::Scalar(0, 255, 0), -1);
        }

        // Display the frame
        cv::imshow("Face Detection Viewer", display_frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr face_detected_sub_;
    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr face_roi_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr face_landmarks_sub_;

    cv::Mat current_frame_;
    int face_detected_ = 0;
    cv::Rect face_roi_;
    std::vector<cv::Point> face_landmarks_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceDetectorViewer>());
    rclcpp::shutdown();
    return 0;
}