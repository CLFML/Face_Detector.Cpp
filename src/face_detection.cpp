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

#include "face_detection.hpp"

#include "tensorflow/lite/kernels/register.h"
#include "opencv2/imgproc.hpp"

namespace CLFML::FaceDetection
{

    cv::Rect2f FaceDetector::get_roi_from_model_box(int index)
    {
        /* Get the corresponding scaling anchor from our pregenerated Anchor list*/
        cv::Rect2f scaling_anchor = m_anchors[index];

        /* As the model_boxes are center_aligned we need to convert our top_left aligned scaling anchor to a center_aligned scaling anchor */
        cv::Point_<float> center_of_scaling_anchor = (scaling_anchor.tl() + scaling_anchor.br()) / 2;

        /*
         * The 16 offset is needed because the output of the model is 896 boxes * 16 outputs
         * Of which the first 4 outputs contains the center_x coordinate, center_y coordinate, width and height of the model box
         * The other 12 outputs probably contain landmarks of the Left eye, Right Eye, Nose tip, Mouth, Left eye tragion and Right eye tragion
         */
        const int box_index = index * 16;
        float box_center_x = m_model_regressors[box_index];
        float box_center_y = m_model_regressors[box_index + 1];
        float box_width = m_model_regressors[box_index + 2];
        float box_height = m_model_regressors[box_index + 3];

        /*
         *  The model boxes output are scaled to the 128x128 image, to make scaling to our input frame (of higher resolution easier)
         *  We need to align the model box to our grid of anchors!
         *  This grid of Anchors can later be aligned to our input image and make it possible to get a scaled ROI on the input image
         */
        box_center_x = box_center_x / m_input_frame_size_x * scaling_anchor.width + center_of_scaling_anchor.x;
        box_center_y = box_center_y / m_input_frame_size_y * scaling_anchor.height + center_of_scaling_anchor.y;
        box_width = box_width / m_input_frame_size_x * scaling_anchor.width;
        box_height = box_height / m_input_frame_size_y * scaling_anchor.height;

        /*
         * The OpenCV library does not allow us to define a rectangle with center x and center y coordinates.
         * Instead it forces us to define the rectangle from a top_left coordinate. So we have to calculate this first!
         */
        const float box_top_left_x = box_center_x - box_width / 2;
        const float box_top_left_y = box_center_y - box_height / 2;

        return cv::Rect2f(box_top_left_x, box_top_left_y, box_width, box_height);
    }

    int FaceDetector::get_best_roi_from_model_result()
    {
        /*
         * The model outputs boxes (which align to our anchor grid) and a confidence score for each of those boxes!
         * Obviously the box which has highest confidence score contains our face.
         * To get the best box we need to itterate through the array and see if it surpasses the threshold (which can be set by user).
         * When threshold is surpassed we check it with a previous surpassed score to determine whether this detected ROI score is better.
         *
         * At the end we process the box and convert it to a ROI aligned to our anchor grid!
         */
        float best_score;
        int roi_detected = -1;
        int index_of_best_score;
        for (int i = 0; i < NUM_OF_FACE_DETECTOR_OUTPUT_BOXES; i++)
        {
            if (m_model_classifiers.at(i) > std::max(m_det_threshold, best_score))
            {
                best_score = m_model_classifiers.at(i);
                index_of_best_score = i;
                roi_detected = 0;
            }
        }

        /* Process the model_box if a roi was detected and get the face roi scaled to our anchor grid */
        if (roi_detected == 0)
        {
            m_roi_from_model = get_roi_from_model_box(index_of_best_score);
        }

        return roi_detected;
    }

    cv::Rect FaceDetector::scale_roi_to_image(cv::Mat &image)
    {
        int image_width = image.size().width;
        int image_height = image.size().height;

        cv::Point_<float> box_center = (m_roi_from_model.tl() + m_roi_from_model.br()) / 2;

        /* Scale the box x,y coordinates by multiplying with the image_width and height */
        box_center.x *= image_width;
        box_center.y *= image_height;

        float box_width = m_roi_from_model.width * image_width * 1.5f;
        float box_height = m_roi_from_model.height * image_height * 2.0f;

        const int top_left_x = (int)box_center.x - box_width / 2;
        const int top_left_y = (int)box_center.y - box_height / 2;

        return cv::Rect(top_left_x, top_left_y, (int)box_width, (int)box_height);
    }

    enum output_tensor_id
    {
        OUTPUT_TENSOR_REGRESSOR,
        OUTPUT_TENSOR_CLASSIFIER
    };

    void FaceDetector::get_regressor()
    {
        size_t num_of_bytes = m_output_tensors.at(OUTPUT_TENSOR_REGRESSOR)->bytes;
        size_t num_of_floats = num_of_bytes / sizeof(float);

        memcpy(&(m_model_regressors.at(0)), m_output_tensors.at(OUTPUT_TENSOR_REGRESSOR)->data.f, num_of_bytes);
    }

    void FaceDetector::get_classifier()
    {
        size_t num_of_bytes = m_output_tensors.at(OUTPUT_TENSOR_CLASSIFIER)->bytes;
        size_t num_of_floats = num_of_bytes / sizeof(float);

        std::vector<float> inference(num_of_floats);
        memcpy(&(m_model_classifiers.at(0)), m_output_tensors.at(OUTPUT_TENSOR_CLASSIFIER)->data.f, num_of_bytes);
    }

    /**
     *
     * @brief This function converts images with other color-spaces to RGB.
     *        As the model expects RGB formatted images.
     * @param in The image to convert to RGB,
     *           Can be CV_8UC3; 8-bit int with 3 channels
     *           Or CV_8UC4; 8-bit int with 4 channels
     * @return RGB formatted frame
     *
     */
    cv::Mat convert_image_to_rgb(const cv::Mat &in)
    {
        cv::Mat rgb_frame;
        int frame_color_type = in.type();

        switch (frame_color_type)
        {
        case CV_8UC3:
        {
            cv::cvtColor(in, rgb_frame, cv::COLOR_BGR2RGB);
            break;
        }
        case CV_8UC4:
        {
            cv::cvtColor(in, rgb_frame, cv::COLOR_BGRA2RGB);
            break;
        }
        default:
        {
            fprintf(stderr, "ERROR: Image type %d is not supported by the face_detector library! \n", frame_color_type);
            exit(1);
        }
        };
        return rgb_frame;
    }

    cv::Mat FaceDetector::preprocess_image(const cv::Mat &in)
    {
        cv::Mat preprocessed_frame = convert_image_to_rgb(in);
        cv::Size input_frame_size = cv::Size(m_input_frame_size_x, m_input_frame_size_y);
        cv::resize(preprocessed_frame, preprocessed_frame, input_frame_size);
#ifdef FACE_DETECTOR_ENABLE_CORAL_SUPPORT
        if (m_delegate_type == face_detector_delegate::CPU)
        {
            const double alpha = 1 / 127.5f;
            const double beta = -127.5f / 127.5f;
            preprocessed_frame.convertTo(preprocessed_frame, CV_32FC3, alpha, beta);
        }
#else
        const double alpha = 1 / 127.5f;
        const double beta = -127.5f / 127.5f;
        preprocessed_frame.convertTo(preprocessed_frame, CV_32FC3, alpha, beta);
#endif
        return preprocessed_frame;
    }

    void FaceDetector::load_image(cv::Mat &camera_frame)
    {
        /* Convert image to 128x128 pixels image with CV32_FC3 format */
        cv::Mat preprocessed_image = preprocess_image(camera_frame);

        /* Copy the image data to the input tensor, which feeds it into the model */
        memcpy(m_input_tensor->data.f, preprocessed_image.data, m_input_tensor->bytes);

        /* Run inference! */
        m_model_interpreter->Invoke();

        /* Get model boxes */
        get_regressor();

        /* Get model boxes confidence score */
        get_classifier();

        /* Calculate ROI (aligned to Anchor grid) from model outputs */
        m_roi_detected = get_best_roi_from_model_result();

        /* Check if ROI is within the captured camera_frame and scale to input frame */
        if (m_roi_detected != -1)
        {
            m_roi = scale_roi_to_image(camera_frame);
        }
        else
        {
            m_roi = cv::Rect();
        }
    }

    void FaceDetector::generate_anchor_grid()
    {
        /*
         * The model outputs grid-aligned boxes scaled to a 128x128 input image.
         * Besides this the model uses two different grid-types;
         *   1x 16x16x2 (16x16 grid with two sections per grid-box)
         *   1x 8x8x6 (8x8 grid with six sections per grid-box)
         *
         * By pregenerating this grid of 896 (16*16*2 + 8*8*6) boxes (also called anchors), we can scale it easily to our higher resolution input-image.
         * The array m_anchors containing all these boxes directly maps to the model_output.
         */
        const std::array<std::array<int, 2>, 2> grid_sizes_and_layers = {{
            {16, 2}, /* 16x16 grid with 2-layers per box*/
            {8, 6}   /* 8x8 grid with 6-layers per box */
        }};
        /* Boxes are square and have a size of {1,1} (so that it can be easily scaled by multiplication) */
        const float box_size_height = 1.0f;
        const float box_size_width = box_size_height;
        const float box_size_half = box_size_height / 2;

        size_t anchor_index = 0;
        for (int i = 0; i < grid_sizes_and_layers.size(); ++i)
        {
            int grid_size = grid_sizes_and_layers[i][0];
            int number_of_anchers_per_layer = grid_sizes_and_layers[i][1];
            float size_inverted = 1.0f / grid_size;
            for (int box_y = 0; box_y < grid_size; ++box_y)
            {
                /* Generate each row of Anchor boxes */
                for (int box_x = 0; box_x < grid_size; ++box_x)
                {
                    /* Scale the box_center_x and box_center_y coordinate to our wanted grid size! */
                    float box_center_x = (box_x + box_size_half) * size_inverted;
                    float box_center_y = (box_y + box_size_half) * size_inverted;

                    /* OpenCV Rectangles are defined from the top_left coordinate not the center coordinate */
                    float box_top_left_x = box_center_x - box_size_half;
                    float box_top_left_y = box_center_y - box_size_half;

                    cv::Rect2f anchor(box_top_left_x, box_top_left_y, box_size_width, box_size_height);

                    /* Generate the layers */
                    for (int ancher_num = 0; ancher_num < number_of_anchers_per_layer; ++ancher_num)
                    {
                        m_anchors.at(anchor_index++) = anchor;
                    }
                }
            }
        }
    }

    FaceDetector::FaceDetector(const float det_threshold) : m_det_threshold(det_threshold)
    {
        /* This generates our anchor grid which is later used to scale model results to our input image */
        generate_anchor_grid();
    }

    void FaceDetector::load_model(const std::string model_path, const face_detector_delegate delegate_type, const uint8_t num_of_threads)
    {
        m_delegate_type = delegate_type;
#ifdef FACE_DETECTOR_ENABLE_CORAL_SUPPORT
        if (delegate_type == face_detector_delegate::CORAL_TPU)
        {
            m_edgetpu_context = edgetpu::EdgeTpuManager::GetSingleton()->OpenDevice();
            if (!m_edgetpu_context)
            {
                fprintf(stderr, "No coral found!\n");
                exit(1);
            }
        }
#endif

        /* Load the model in to memory */
        m_model = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
        if (m_model == nullptr)
        {
            fprintf(stderr, "File \"%s\" ERROR: Can't build flatbuffer from model: %s \n", __FILE__, model_path.c_str());
            exit(1);
        }

        /*
         * We want to use the default tensorflow operations (for CPU inference):
         * See: https://www.tensorflow.org/lite/guide/ops_compatibility
         */
        tflite::ops::builtin::BuiltinOpResolver resolver;

#ifdef FACE_DETECTOR_ENABLE_CORAL_SUPPORT
        if (delegate_type == face_detector_delegate::CORAL_TPU)
        {
            /*
             * The custom tuned model for tpu usage uses custom tpu operations!
             * This line of code links this custom command to our coral edge tpu,
             * as our resolver/interpreter does not know how to handle this custom operation by default
             */
            resolver.AddCustom(edgetpu::kCustomOp, edgetpu::RegisterCustomOp());
        }
#endif

        /*
         * Build the model interpreter with our model and the resolver;
         * This gives a handle and saves it into m_model_intepreter which is later used for doing inference
         */
        if (tflite::InterpreterBuilder(*m_model, resolver)(&m_model_interpreter) != kTfLiteOk)
        {
            fprintf(stderr, "File \"%s\" ERROR: Can't initialize the interpreter \n", __FILE__);
            exit(1);
        }

#ifdef FACE_DETECTOR_ENABLE_CORAL_SUPPORT
        if (delegate_type == face_detector_delegate::CORAL_TPU)
        {
            /*
             * We give our model interpreter a handle to the edgetpu context,
             * so that the interpreter can now communicate with our edgetpu!
             */
            m_model_interpreter->SetExternalContext(kTfLiteEdgeTpuContext, m_edgetpu_context.get());
        }
        else
        {
            /*
             * We can set the amount of CPU threads we want to dedicate to our model_interpreter engine.
             * This is only useful for CPU inference,
             * Using multiple threads with TPU inference will slow down the program, due to synchronisation between threads!
             * Default = 4 threads
             */
            m_model_interpreter->SetNumThreads(num_of_threads);
        }
#else
        /*
         * We can set the amount of CPU threads we want to dedicate to our model_interpreter engine.
         * This is only useful for CPU inference,
         * Using multiple threads with TPU inference will slow down the program, due to synchronisation between threads!
         * Default = 4 threads
         */
        m_model_interpreter->SetNumThreads(num_of_threads);
#endif

        /* Allocate memory for model inference */
        if (m_model_interpreter->AllocateTensors() != kTfLiteOk)
        {
            fprintf(stderr, "File \"%s\" ERROR: Can't allocate tensors for face detector model interpreter \n", __FILE__);
            exit(1);
        }

        /*
         * Get the amount of input tensors (vectors) of the model, which should be one
         * As this is where the 128x128 input frame will be copied to before doing inference
         */
        const std::vector<int> &inputs = m_model_interpreter->inputs();

        /* Save the handle to this input vector for later */
        m_input_tensor = m_model_interpreter->tensor(inputs.at(0));

        /* Get the input frame size (should be 128x128 pixels!) */
        m_input_frame_size_x = m_input_tensor->dims->data[1];
        m_input_frame_size_y = m_input_tensor->dims->data[2];

        /*
         * Get the amount of output tensors of the model, which should be two;
         *  - one for the list of regressors(boxes)
         *  - one for the list of classifiers (confidence score)
         */
        const std::vector<int> &outputs = m_model_interpreter->outputs();

        if (outputs.size() != m_output_tensors.size())
        {
            fprintf(stderr, "File \"%s\" ERROR: Model tensor quantity does not match expected tensors! \n", __FILE__);
            exit(1);
        }

        /* Save the output tensor handles for later! (With bound-check)*/
        for (uint8_t i = 0; i < m_output_tensors.size(); i++)
        {
            int tensor_index = outputs.at(i);
            m_output_tensors.at(i) = m_model_interpreter->tensor(tensor_index);
        }
    }
    cv::Rect FaceDetector::get_face_roi()
    {
        return m_roi;
    }

    int FaceDetector::detected()
    {
        return m_roi_detected;
    }

    FaceDetector::~FaceDetector()
    {
        m_model_interpreter.reset();
#ifdef FACE_DETECTOR_ENABLE_CORAL_SUPPORT
        if (m_delegate_type == face_detector_delegate::CORAL_TPU)
        {
            m_edgetpu_context.reset();
        }
#endif
    }
}
