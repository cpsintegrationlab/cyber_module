#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo
{
namespace safety_layer
{
class LidarComponent final : public cyber::TimerComponent
{
public:

	LidarComponent();

	bool
	Init() override;

	bool
	Proc() override;

private:

	void
	OnPointCloudMessage(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	std::shared_ptr<cyber::Reader<drivers::PointCloud>> point_cloud_reader_;

	unsigned long frame_counter_;
};

CYBER_REGISTER_COMPONENT(LidarComponent)
} // safety_layer
} // apollo
