#include <iostream>
#include <face_detection.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

#define MODEL_PATH FACE_DETECTOR_MODEL_DIR "/CPU/face_detection.tflite"

int main(int argc, char *argv[])
{
    /* Initialize camera */
    const uint8_t camera_index = 0;
    const uint16_t camera_fps = 30;
    const uint32_t width = 640;
    const uint32_t height = 480;
    cv::VideoCapture cam(camera_index);

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

    /* Load model and initialize inference runtime */
    det.load_model(MODEL_PATH);

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
        cv::putText(cam_frame, top_left_text, cv::Point(20,70), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 2);

        /* Get the face roi rectangle */
        cv::Rect face_roi = det.get_face_roi();

        /* Get the head pose */
        cv::Vec3f head_pose = det.get_head_pose();

        /* Convert the head pose to a string */
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "Roll: " << head_pose[0] * 180.0 / CV_PI << "°, "
            << "Pitch: " << head_pose[1] * 180.0 / CV_PI << "°, "
            << "Yaw: " << head_pose[2] * 180.0 / CV_PI << "°";
        const std::string m_head_pose_string = oss.str();

        /* Draw the head pose string on the captured camera frame */
        cv::putText(cam_frame, m_head_pose_string, cv::Point(20,90), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 255), 2);

        /* Draw the face roi rectangle on the captured camera frame */
        cv::rectangle(cam_frame, face_roi, cv::Scalar(0, 255, 0), 2); // Green rectangle will be drawn around detected face

        det.draw_keypoints(cam_frame);
        
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