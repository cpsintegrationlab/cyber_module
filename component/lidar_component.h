#include <memory>
#include <boost/property_tree/ptree.hpp>

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

	~LidarComponent();

	bool
	Init() override;

	bool
	Proc() override;

private:

	void
	ProcessGroundTruth3D(const std::shared_ptr<common::Detection3DArray> ground_truth_3d_message);

	void
	ProcessPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	void
	LogGroundTruth3D(const std::shared_ptr<common::Detection3DArray> ground_truth_3d_message);

	void
	LogPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	std::shared_ptr<cyber::Reader<common::Detection3DArray>> ground_truth_3d_reader_;
	std::shared_ptr<cyber::Reader<drivers::PointCloud>> point_cloud_reader_;
	std::shared_ptr<DepthClustering> depth_clustering_;

	const std::string ground_truth_3d_log_file_name_ = "/apollo/data/lidar/ground_truth_3d.json";
	boost::property_tree::ptree ground_truth_3d_log_file_tree_;
	std::ofstream ground_truth_3d_log_file_;
	unsigned long frame_counter_;
	bool log_;
};

CYBER_REGISTER_COMPONENT(LidarComponent)
} // safety_layer
} // apollo
