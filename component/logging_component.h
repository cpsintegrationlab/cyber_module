#include <boost/property_tree/ptree.hpp>
#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/component/timer_component.h"

namespace apollo
{
namespace safety_layer
{
class LoggingComponent final : public cyber::TimerComponent
{
public:

	LoggingComponent();

	~LoggingComponent();

	bool
	Init() override;

	bool
	Proc() override;

private:

	void
	createLogDirectoryPointCloud();

	void
	LogGroundTruth3D(const std::shared_ptr<perception::PerceptionObstacles> message);

	void
	LogPointCloud(const std::shared_ptr<drivers::PointCloud> message);

    void
	writeLogGroundTruth3D();

	std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>> reader_ground_truth_3d_;
	std::shared_ptr<cyber::Reader<drivers::PointCloud>> reader_point_cloud_;
	const std::string channel_name_reader_ground_truth_3d_;
	const std::string channel_name_reader_point_cloud_;

	bool log_;
    const std::string log_directory_name_point_cloud_;
	const std::string log_file_name_ground_truth_3d_;
	boost::property_tree::ptree log_file_tree_ground_truth_3d_;
};

CYBER_REGISTER_COMPONENT(LoggingComponent)
}   /* safety_layer */
}   /* apollo */
