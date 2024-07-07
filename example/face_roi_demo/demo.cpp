#include <iostream>
#include <face_detection.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

#define MODEL_PATH FACE_DETECTOR_MODEL_DIR "/CPU/face_detection.tflite"

int main(int argc, char *argv[])
{
    CLFML::FaceDetection::FaceDetector det;
    det.load_model(MODEL_PATH);
    cv::Mat cam_frame;

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

    cv::namedWindow("Display window", cv::WINDOW_NORMAL);
    cv::resizeWindow("Display window", width, height);

    while (true)
    {
        if (!cam.read(cam_frame))
        {
            continue;
        }
        det.load_image(cam_frame);
        cv::Rect face_roi = det.get_face_roi();
        cv::rectangle(cam_frame, face_roi, cv::Scalar(0, 255, 0), 2); // Green rectangle will be drawn around detected face
        cv::imshow("Display window", cam_frame);
        // Break the loop on 'q' key press
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    return 0;
}