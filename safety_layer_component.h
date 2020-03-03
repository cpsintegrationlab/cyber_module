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
#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/perception/camera/common/camera_frame.h"

namespace apollo
{
namespace safety_layer
{
class SafetyLayerComponent final : public cyber::TimerComponent
{
public:

	SafetyLayerComponent();

	bool
	Init() override;

	bool
	Proc() override;

private:

	int
	InitCameraFrame();

	void
	OnDepthImageMessage(const std::shared_ptr<drivers::Image> depth_image_message);

	perception::camera::DataProvider::InitOptions depth_camera_data_provider_init_options_;
	perception::camera::CameraFrame depth_camera_frame_;

	std::shared_ptr<perception::camera::DataProvider> depth_camera_data_provider_;
	std::shared_ptr<cyber::Reader<drivers::Image>> depth_image_reader_;

	int image_width_;
  	int image_height_;
	int gpu_id_;

	bool enable_undistortion_;
	bool show_depth_image_;
	bool log_depth_image_;

	std::string depth_camera_name_;
};

CYBER_REGISTER_COMPONENT(SafetyLayerComponent)
} // safety_layer
} // apollo
