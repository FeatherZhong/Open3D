// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "Open3D/IO/Sensor/AzureKinect/AzureKinectRecorder.h"

#include <assert.h>
#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <ctime>
#include <iostream>

#include <k4a/k4a.h>
#include <k4arecord/record.h>

#include "Open3D/Geometry/RGBDImage.h"
#include "Open3D/IO/Sensor/AzureKinect/K4aPlugin.h"
#include "Open3D/IO/Sensor/AzureKinect/MKVReader.h"
#include "Open3D/Visualization/Utility/ColorMap.h"
#include "Open3D/Visualization/Visualizer/VisualizerWithKeyCallback.h"

namespace open3d {
namespace io {

AzureKinectRecorder::AzureKinectRecorder(
        const AzureKinectSensorConfig& sensor_config, size_t sensor_index)
    : RGBDRecorder(),
      sensor_(AzureKinectSensor(sensor_config)),
      device_index_(sensor_index) {}

AzureKinectRecorder::~AzureKinectRecorder() { CloseRecord(); }

bool AzureKinectRecorder::InitSensor() {
    return sensor_.Connect(device_index_);
}

bool AzureKinectRecorder::OpenRecord(const std::string& filename) {
    if (!is_record_created_) {
        if (K4A_FAILED(k4a_plugin::k4a_record_create(
                    filename.c_str(), sensor_.device_,
                    sensor_.sensor_config_.ConvertToNativeConfig(),
                    &recording_))) {
            utility::LogWarning("Unable to create recording file: {}",
                                filename);
            return false;
        }
        if (K4A_RESULT_SUCCEEDED != k4a_plugin::k4a_record_add_imu_track(recording_)) {
            utility::LogError("Unable to write IMU track\n");
        }

        if (K4A_FAILED(k4a_plugin::k4a_record_write_header(recording_))) {
            utility::LogWarning("Unable to write header");
            return false;
        }
        utility::LogInfo("Writing to header");

        is_record_created_ = true;
    }
    return true;
}

bool AzureKinectRecorder::CloseRecord() {
    if (is_record_created_) {
        utility::LogInfo("Saving recording...");
        if (K4A_FAILED(k4a_plugin::k4a_record_flush(recording_))) {
            utility::LogWarning("Unable to flush record file");
            return false;
        }
        k4a_plugin::k4a_record_close(recording_);
        utility::LogInfo("Done");

        is_record_created_ = false;
    }
    return true;
}

std::shared_ptr<geometry::RGBDImage> AzureKinectRecorder::RecordFrame(
        bool write, bool enable_align_depth_to_color) {
    k4a_capture_t capture = sensor_.CaptureRawFrame();
    if (capture != nullptr && is_record_created_ && write) {
        if (K4A_FAILED(k4a_plugin::k4a_record_write_capture(recording_,
                                                            capture))) {
            utility::LogError("Unable to write to capture");
        }
    }

    auto im_rgbd = AzureKinectSensor::DecompressCapture(
            capture, enable_align_depth_to_color
                             ? sensor_.transform_depth_to_color_
                             : nullptr);
    if (im_rgbd == nullptr) {
        utility::LogInfo("Invalid capture, skipping this frame");
        return nullptr;
    }
    k4a_plugin::k4a_capture_release(capture);
    k4a_wait_result_t imu_result = K4A_WAIT_RESULT_SUCCEEDED;
    do{
        k4a_imu_sample_t sample;
        imu_result = k4a_plugin::k4a_device_get_imu_sample(sensor_.device_, &sample, 0);

        if (imu_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cerr << "Runtime error: k4a_imu_get_sample() returned TIMEOUT" << imu_result << std::endl;
        }
        else if (imu_result != K4A_WAIT_RESULT_SUCCEEDED)
        {
            std::cerr << "Runtime error: k4a_imu_get_sample() returned " << imu_result << std::endl;
        }
        if (is_record_created_ && write)
        {
            k4a_result_t write_result = k4a_plugin::k4a_record_write_imu_sample(recording_, sample);
            utility::LogInfo("Writing IMU {},{},{}",sample.acc_sample.xyz.x,sample.acc_sample.xyz.y,sample.acc_sample.xyz.z);
            if (K4A_FAILED(write_result))
            {
                std::cerr << "Runtime error: k4a_record_write_imu_sample() returned " << write_result << std::endl;
            }
        }
    }while (imu_result == K4A_WAIT_RESULT_SUCCEEDED);


    return im_rgbd;
}
}  // namespace io
}  // namespace open3d
