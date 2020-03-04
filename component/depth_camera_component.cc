#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "modules/perception/base/image_8u.h"
#include "modules/safety_layer/component/depth_camera_component.h"

namespace apollo
{
namespace safety_layer
{
DepthCameraComponent::DepthCameraComponent() : depth_image_reader_(nullptr), 
	image_width_(1920), image_height_(1080), gpu_id_(0), enable_undistortion_(false),
	show_depth_image_(true), log_depth_image_(false), frame_counter_(0),
	depth_camera_name_("depth_camera")
{
	if (InitCameraFrame() != cyber::SUCC)
	{
		AERROR << "Failed to initialize camera frame.";
	}
}

bool
DepthCameraComponent::Init()
{
	depth_image_reader_ = node_->CreateReader<drivers::Image>(
		"/apollo/sensor/camera/depth/image");

	return true;
}

bool
DepthCameraComponent::Proc()
{
	if (depth_image_reader_ == nullptr)
	{
		AERROR << "Depth image reader missing.";
		return false;
	}

	depth_image_reader_->Observe();
	const auto& depth_image = depth_image_reader_->GetLatestObserved();

	if (depth_image == nullptr)
	{
		return false;
	}

	OnDepthImageMessage(depth_image);

	return true;
}

int
DepthCameraComponent::InitCameraFrame()
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
DepthCameraComponent::OnDepthImageMessage(const
	std::shared_ptr<drivers::Image> depth_image_message)
{
	if (!depth_camera_frame_.data_provider)
	{
		AERROR << "Data provider for " << depth_camera_name_ << " missing.";
		return;
	}

	AERROR << "Received depth image message.";

	cv::Mat depth_image(image_height_, image_width_, CV_8UC1, cv::Scalar(0));
	perception::base::Image8U depth_image_8u(image_height_, image_width_, perception::base::Color::GRAY);
	perception::camera::DataProvider::ImageOptions depth_image_options;

	depth_camera_frame_.timestamp = depth_image_message->measurement_time();
	depth_camera_frame_.data_provider->FillImageData(image_height_, image_width_,
		reinterpret_cast<const uint8_t*>(depth_image_message->data().data()),
		depth_image_message->encoding());
	
	depth_image_options.target_color = perception::base::Color::GRAY;
	depth_camera_frame_.data_provider->GetImage(depth_image_options, &depth_image_8u);
	memcpy(depth_image.data, depth_image_8u.cpu_data(), depth_image_8u.total() * sizeof(uint8_t));

	bitwise_not(depth_image, depth_image);
	depth_image.convertTo(depth_image, CV_16UC1);
	depth_image *= 256;

	if (show_depth_image_)
	{
		cv::imshow("Depth Camera", depth_image);
		cvWaitKey(30);
	}
	
	if (log_depth_image_)
	{
		cv::imwrite("/apollo/data/camera/depth/" + std::to_string(frame_counter_) + ".png", depth_image);
		frame_counter_ ++;
	}

	return;
}
} // safety_layer
} // apollo