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
class DepthCameraComponent final : public cyber::TimerComponent
{
public:

	DepthCameraComponent();

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

	unsigned long frame_counter_;

	std::string depth_camera_name_;
};

CYBER_REGISTER_COMPONENT(DepthCameraComponent)
} // safety_layer
} // apollo
