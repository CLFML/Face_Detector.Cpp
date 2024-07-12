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
* Author:          Victor Hogeweij <hogeweyv@gmail.com>
*/

#include <iostream>
#include <face_detection.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <thread>


/* Uncomment this when using a Coral TPU, this is the special quantized model for TPU usage! */
//#define MODEL_PATH_TPU FACE_DETECTOR_MODEL_DIR "/Coral/face_detection.tflite"

/* Leave this uncommented when using CPU inference, this is uncommented by default */
#define MODEL_PATH_CPU FACE_DETECTOR_MODEL_DIR "/CPU/face_detection.tflite"


int main(int argc, char *argv[])
{
    /* Initialize camera */
    const uint8_t camera_index = 0;
    const uint16_t camera_fps = 30;
    const uint32_t width = 640;
    const uint32_t height = 480;
    cv::VideoCapture cam(camera_index, cv::CAP_V4L2);

    if (cam.isOpened() == false)
    {
        fprintf(stderr, "ERROR: Cannot open camera!\n");
        exit(1);
    }

    cam.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cam.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cam.set(cv::CAP_PROP_FRAME_COUNT, camera_fps);

    /* Initialize face detector library */
    CLFML::FaceDetection::FaceDetector det;


    /* UNCOMMENT this line when using a TPU*/
    /* Load model and initialize inference runtime with Coral TPU delegate */
    //det.load_model(MODEL_PATH_TPU, CLFML::FaceDetection::face_detector_delegate::CORAL_TPU);
    
    /* COMMENT this line when using a TPU, when using CPU leave it uncommented! As this will prepare the library for CPU inference! */
    det.load_model(MODEL_PATH_CPU);


    /* Create window to show the face roi */
    cv::namedWindow("Display window", cv::WINDOW_NORMAL);
    cv::resizeWindow("Display window", width, height);

    cv::Mat cam_frame;
    while (true)
    {
        /* If no frame captured? Try again! */
        if (!cam.read(cam_frame))
        {
            continue;
        }

        /* Load image into model and do inference! */
        det.load_image(cam_frame);

        /* Get face_detected value */
        const int face_detected = det.detected() + 1; // +1 because it returns -1 for no face and 0 for face detected!

        /* Convert the face_detected integer to string */
        const std::string top_left_text = "Detected: " + std::to_string(face_detected);

        /* Draw (red) text in corner of frame telling whether a face has been detected; 0 no face, 1 face has been detected */
        cv::putText(cam_frame, top_left_text, cv::Point(20, 70), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 2);

        /* Get the face roi rectangle */
        cv::Rect face_roi = det.get_face_roi();

        /* Draw the face roi rectangle on the captured camera frame */
        cv::rectangle(cam_frame, face_roi, cv::Scalar(0, 255, 0), 2); // Green rectangle will be drawn around detected face

        /* Update the window with the newly made image */
        cv::imshow("Display window", cam_frame);

        /* Break the loop on 'q' key press */
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    return 0;
}