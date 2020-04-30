#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo
{
namespace safety_layer
{
class DepthClustering;

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
	ProcessPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	void
	LogPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	std::shared_ptr<cyber::Reader<drivers::PointCloud>> point_cloud_reader_;
	std::shared_ptr<DepthClustering> depth_clustering_;

	unsigned long frame_counter_;
	bool log_;
};

CYBER_REGISTER_COMPONENT(LidarComponent)
} // safety_layer
} // apollo
