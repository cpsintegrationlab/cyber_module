#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "lgsvl_pkgs/lgsvl_msgs/proto/detection3darray.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/safety_layer/lib/depth_clustering/src/src/api/depth_clustering.h"

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
	Process3DGroundTruth(const std::shared_ptr<common::Detection3DArray> 3d_ground_truth_message);

	void
	ProcessPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	void
	Log3DGroundTruth(const std::shared_ptr<common::Detection3DArray> 3d_ground_truth_message);

	void
	LogPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	std::shared_ptr<cyber::Reader<common::Detection3DArray>> 3d_ground_truth_reader_;
	std::shared_ptr<cyber::Reader<drivers::PointCloud>> point_cloud_reader_;
	std::shared_ptr<DepthClustering> depth_clustering_;

	unsigned long frame_counter_;
	bool log_;
};

CYBER_REGISTER_COMPONENT(LidarComponent)
} // safety_layer
} // apollo
