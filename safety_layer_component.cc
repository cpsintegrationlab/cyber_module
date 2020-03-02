/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "modules/perception/base/image_8u.h"
#include "modules/safety_layer/safety_layer_component.h"

namespace apollo
{
namespace safety_layer
{
SafetyLayerComponent::SafetyLayerComponent() : depth_image_reader_(nullptr),
	image_width_(1920), image_height_(1080), gpu_id_(0), enable_undistortion_(false),
	depth_camera_name_("depth_camera")
{
	if (InitCameraFrame() != cyber::SUCC)
	{
		AERROR << "Failed to initialize camera frame.";
	}
}

bool
SafetyLayerComponent::Init()
{
	depth_image_reader_ = node_->CreateReader<drivers::Image>(
		"/apollo/sensor/camera/depth/image");
	return true;
}

bool
SafetyLayerComponent::Proc()
{
	if (depth_image_reader_ == nullptr)
	{
		AERROR << "Depth image reader missing.";
		return false;
	}

	depth_image_reader_->Observe();
	const auto &depth_image = depth_image_reader_->GetLatestObserved();

	if (depth_image == nullptr)
	{
		return false;
	}

	OnDepthImageMessage(depth_image);

	return true;
}

int
SafetyLayerComponent::InitCameraFrame()
{
	depth_camera_data_provider_init_options_.image_height = image_height_;
	depth_camera_data_provider_init_options_.image_width = image_width_;
	depth_camera_data_provider_init_options_.device_id = gpu_id_;
	depth_camera_data_provider_init_options_.do_undistortion = enable_undistortion_;
	depth_camera_data_provider_init_options_.sensor_name = depth_camera_name_;

	depth_camera_data_provider_ = std::make_shared<perception::camera::DataProvider>();

	if (!depth_camera_data_provider_)
	{
		AERROR << "Data provider for " << depth_camera_name_ << " missing.";
		return cyber::FAIL;
	}

	if (!depth_camera_data_provider_->Init(depth_camera_data_provider_init_options_))
	{
		AERROR << "Failed to initialize data provider for " << depth_camera_name_;
		return cyber::FAIL;
	}

	depth_camera_frame_.data_provider = depth_camera_data_provider_.get();

	return cyber::SUCC;
}

void
SafetyLayerComponent::OnDepthImageMessage(const
	std::shared_ptr<drivers::Image> depth_image_message)
{
	if (!depth_camera_frame_.data_provider)
	{
		AERROR << "Data provider for " << depth_camera_name_ << " missing.";
		return;
	}

	cv::Mat depth_image(image_height_, image_width_, CV_8UC3, cv::Scalar(0, 0, 0));
	perception::base::Image8U depth_image_8u(image_height_, image_width_, perception::base::Color::RGB);
	perception::camera::DataProvider::ImageOptions depth_image_options;

	AERROR << "Received depth image message.";

	depth_camera_frame_.timestamp = depth_image_message->measurement_time();
	depth_camera_frame_.data_provider->FillImageData(image_height_, image_width_,
		reinterpret_cast<const uint8_t*>(depth_image_message->data().data()),
		depth_image_message->encoding());
	
	depth_image_options.target_color = perception::base::Color::BGR;
	depth_camera_frame_.data_provider->GetImage(depth_image_options, &depth_image_8u);
	memcpy(depth_image.data, depth_image_8u.cpu_data(), depth_image_8u.total() * sizeof(uint8_t));

	cv::imshow("Depth Camera", depth_image);
	cvWaitKey(30);

	return;
}
} // safety_layer
} // apollo