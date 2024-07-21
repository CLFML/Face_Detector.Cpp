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
* Author:          Victor Hogeweij <Hoog-V>
*
*/

#ifndef FACE_DETECTION_HPP
#define FACE_DETECTION_HPP
#include <string>
#include <vector>
#include <array>
#include "opencv2/core.hpp"
#include "tensorflow/lite/interpreter.h"
#include <tensorflow/lite/model.h>

#ifdef FACE_DETECTOR_ENABLE_CORAL_SUPPORT
#include <edgetpu.h>
#endif

namespace CLFML::FaceDetection
{

    /* Number of model_outputs (1x Regressors and 1x Classifiers)*/
    inline constexpr size_t NUM_OF_FACE_DETECTOR_OUTPUT_TENSORS = 2;

    /* Number of model_output boxes */
    inline constexpr size_t NUM_OF_FACE_DETECTOR_OUTPUT_BOXES = 896;

    /* Number of model regressor outputs */
    inline constexpr size_t NUM_OF_FACE_DETECTOR_REGRESSOR_OUTPUTS = NUM_OF_FACE_DETECTOR_OUTPUT_BOXES * 16;

    /* Number of model landmarks */
    inline constexpr size_t NUM_OF_FACE_DETECTOR_LANDMARKS = 6;

    enum class face_detector_delegate
    {
        CPU,
#ifdef FACE_DETECTOR_ENABLE_CORAL_SUPPORT
        CORAL_TPU
#endif
    };

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
         * @param delegate_type The delegate to use for inference (CPU or TPU)
         * @param num_of_threads The number of CPU threads which can be used by the inference runtime
         */
        void load_model(const std::string model_path, const face_detector_delegate delegate_type = face_detector_delegate::CPU, const uint8_t num_of_threads = 4);

        /**
         * @brief Loads image into model and does inference
         * @param frame Any frame which is formatted in CV_8UC3 or CV_8UC4 format
         */
        void load_image(cv::Mat &frame);

        /**
         * @brief Gets the Region of Interest, formatted as a square area (scaled to input image) where a detected face might be in
         * @return Rectangular area which contains a detected face (Top-Left aligned)
         */
        cv::Rect get_face_roi();

        /**
         * @brief Gets the 2D landmarks from the face.
         * @return Array with 6 Facial landmarks;
         *         Index 0: Left Eye
         *         Index 1: Right Eye
         *         Index 2: Nose
         *         Index 3: Mouth
         *         Index 4: Left ear
         *         Index 5: Right ear
         */
        std::array<cv::Point, NUM_OF_FACE_DETECTOR_LANDMARKS> get_face_landmarks();

        /**
         * @brief Determine whether a face was detected
         * @return 0 if face was detected, 1 if no face was detected in input frame
         */
        int detected();

        ~FaceDetector();

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
        TfLiteTensor *m_input_tensor;

        std::array<TfLiteTensor *, NUM_OF_FACE_DETECTOR_OUTPUT_TENSORS> m_output_tensors;

        std::array<float, NUM_OF_FACE_DETECTOR_OUTPUT_BOXES> m_model_classifiers;

        std::array<float, NUM_OF_FACE_DETECTOR_REGRESSOR_OUTPUTS> m_model_regressors;

        /* Intermediary variable which contains a grid-aligned ROI (after model inference) */
        cv::Rect2f m_roi_from_model;

        /* Intermediary variable which contains grid-aligned Landmarks (after model inference) */
        std::array<cv::Point2f, NUM_OF_FACE_DETECTOR_LANDMARKS> m_model_landmarks;

        /*
         * Variables that are used by the getters
         */
        cv::Rect m_roi;
        std::array<cv::Point, NUM_OF_FACE_DETECTOR_LANDMARKS> m_model_landmarks_scaled;
        int m_roi_detected = -1;

        /*
         * Handles to the model and model_inpreter runtime
         */
        std::unique_ptr<tflite::FlatBufferModel> m_model;

        std::unique_ptr<tflite::Interpreter> m_model_interpreter;

#ifdef FACE_DETECTOR_ENABLE_CORAL_SUPPORT
        /*
         * Holds the context of the edgetpu manager; Google thingy that manages all connected edgetpu's
         */
        std::shared_ptr<edgetpu::EdgeTpuContext> m_edgetpu_context;
#endif /* FACE_DETECTOR_ENABLE_CORAL_SUPPORT */

        face_detector_delegate m_delegate_type;

        /**
         * @brief Helper that generates the anchor grid (runs at class construction)
         */
        void generate_anchor_grid();

        /**
         * @brief Helper that preprocesses the input image for inference
         * @return Preprocessed image
         */
        cv::Mat preprocess_image(const cv::Mat &in);

        /**
         * @brief Helper function that calculates the Anchor-grid aligned ROI from model-regressor box
         * @return ROI from model_box aligned to Anchor-grid
         */
        cv::Rect2f get_roi_from_model_box(int index);

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
         * @brief Helper that Aligns the Anchor grid to the image and calculates the corresponding landmark coordinates
         *        (saves them into m_model_landmarks_scaled array!)
         */
        void scale_landmarks_to_image(cv::Mat &image);

        /**
         * @brief Helper that Gets the regressor model outputs and saves them into the m_model_classifiers array
         */
        void get_regressor();

        /**
         * @brief Gets the classifier model outputs and saves them into the m_model_classifiers array
         */
        void get_classifier();
    };

}

#endif /* FACE_DETECTION_HPP */