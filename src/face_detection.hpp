#ifndef FACE_DETECTION_HPP
#define FACE_DETECTION_HPP
#include <string>
#include <vector>
#include <array>
#include "opencv2/core.hpp"
#include "tensorflow/lite/interpreter.h"
#include <tensorflow/lite/model.h>


namespace CLFML::FaceDetection {

/* Number of model_outputs (1x Regressors and 1x Classifiers)*/
inline constexpr size_t NUM_OF_FACE_DETECTOR_OUTPUT_TENSORS = 2;

/* Number of model_output boxes */
inline constexpr size_t NUM_OF_FACE_DETECTOR_OUTPUT_BOXES = 896;

/* Number of model regressor outputs */
inline constexpr size_t NUM_OF_FACE_DETECTOR_REGRESSOR_OUTPUTS = NUM_OF_FACE_DETECTOR_OUTPUT_BOXES * 16;

class FaceDetector
{
public:
    /**
     * @brief Constructor
     * @param det_threshold The sensitivity of the face detector implementation (default=0.75)
     */
    FaceDetector(const float det_threshold = 0.75f);

    /**
     * @brief Loads model and initializes the inference runtime
     * @param model_path Path to the Mediapipe Blazeface face detector model (.tflite) file
     * @param num_of_threads The number of CPU threads which can be used by the inference runtime
     */
    void load_model(const std::string model_path, const uint8_t num_of_threads = 4);

    /**
     * @brief Loads image into model and does inference
     * @param frame Any frame which is formatted in CV_8UC3 or CV_8UC4 format
     */
    void load_image(cv::Mat &frame);

    /** 
     * @brief Gets the Region of Interest, formatted as a square area (scaled to input image) where a detected face might be in
     * @return Rectangular area which contains a detected face
     */
    cv::Rect get_face_roi();

    /**
     * @brief Returns approximate keypoints of the detected face. Left eye (from the observer’s point of view), Right eye,
     * Nose tip, Mouth, Left eye tragion, Right eye tragion.
     * @return std::vector<cv::Point2f> A vector of 2D points representing the keypoints of the detected face.
     */
    std::vector<cv::Point2f> get_face_keypoints();

    /**
     * Draws keypoints on the given image.
     *
     * @param image The image on which keypoints are to be drawn.
     */
    void draw_keypoints(cv::Mat& image);

    /**
     * @brief Determine whether a face was detected
     * @return 0 if face was detected, 1 if no face was detected in input frame
     */
    int detected();

    /**
     * @brief Retrieves the head pose of a detected face.
     * @return The head pose represented as a 3D vector (cv::Vec3f).
     */
    cv::Vec3f get_head_pose();

private:
    /* Detection threshold for model results postprocessing */
    const float m_det_threshold;

    /* Model input frame width and height */
    int32_t m_input_frame_size_x = 128;
    int32_t m_input_frame_size_y = 128;

    /* Array that contains our generated anchor grid (see generate_anchor_grid!) */
    std::array<cv::Rect2f, NUM_OF_FACE_DETECTOR_OUTPUT_BOXES> m_anchors;
    
    /*
    * Model inputs and outputs
    */
    TfLiteTensor* m_input_tensor;
    std::array<TfLiteTensor *, NUM_OF_FACE_DETECTOR_OUTPUT_TENSORS> m_output_tensors;
    std::array<float, NUM_OF_FACE_DETECTOR_OUTPUT_BOXES> m_model_classifiers;
    std::array<float, NUM_OF_FACE_DETECTOR_REGRESSOR_OUTPUTS> m_model_regressors;
    
    /* Intermediary variable which contains a grid-aligned ROI (after model inference) */
    cv::Rect2f m_roi_from_model;

    /* List of 2D keypoints */
    std::vector<cv::Point2f> m_keypoints;

    /* Head pose angles*/
    cv::Vec3f m_head_pose;

    /*
     * Variables that are used by the getters
     */    
    cv::Rect m_roi;
    int m_roi_detected = -1;

    /*
     * Handles to the model and model_inpreter runtime
     */
    std::unique_ptr<tflite::FlatBufferModel> m_model;
    std::unique_ptr<tflite::Interpreter> m_model_interpreter;

    /**
     * @brief Helper that generates the anchor grid (runs at class construction)
     */
    void generate_anchor_grid();

    /**
     * @brief Helper that preprocesses the input image for inference
     * @return Preprocessed image
     */
    cv::Mat preprocess_image(const cv::Mat& in);

    /**
     * @brief Helper function that calculates the Anchor-grid aligned ROI from model-regressor box
     * @return ROI from model_box aligned to Anchor-grid
     */
    cv::Rect2f get_roi_from_model_box(int index);

    /**
     * @brief Helper function that retrieves the keypoints from the model box at the specified index.
     * @param index The index of the model box.
     * @return A vector of cv::Point2f representing the keypoints.
     */
    std::vector<cv::Point2f> get_keypoints_from_model_box(int index);

    /**
     * @brief Helper function that estimates the head pose based on the provided 2D keypoints.
     * @param keypoints A vector of 2D points representing the facial keypoints.
     * @return A 3D vector representing the estimated head pose.
     */
    cv::Vec3f estimate_head_pose(const std::vector<cv::Point2f>& keypoints);
    
    /**
     * @brief Helper function takes a rotation matrix and computes the corresponding Euler angles.
     * @param R The rotation matrix to convert.
     * @return The Euler angles as a `cv::Vec3f` object.
     */
    cv::Vec3f rotation_matrix_to_euler_angle(const cv::Matx33f& R);
    
    /**
     * @brief Helper function that calculates the cross product of two 3D vectors.
     * @param v1 The first 3D vector.
     * @param v2 The second 3D vector.
     * @return The cross product of the two input vectors.
     */
    cv::Vec3f cross_vectors(const cv::Vec3f& v1, const cv::Vec3f& v2);

    /**
     * @brief Helper that Detects and gets the best ROI from the input image, then saves the Anchor-grid aligned ROI to m_roi_from_model
     * @return roi was detected = 0, no roi detected = 1
     */
    int get_best_roi_from_model_result();

    /**
     * @brief Helper that Aligns the Anchor grid to the image and calculates the corresponding ROI coordinates
     * @return Rectangle with the calculated ROI
     */
    cv::Rect scale_roi_to_image(cv::Mat &image);

    /**
     * @brief Helper that Gets the regressor model outputs and saves them into the m_model_classifiers array
     */
    void get_regressor();

    /**
     * @brief Gets the classifier model outputs and saves them into the m_model_classifiers array
     */
    void get_classifier();
};

} // namespace CLFML::FaceDetection

#endif /* FACE_DETECTION_HPP */