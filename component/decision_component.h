#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "lgsvl_pkgs/lgsvl_msgs/proto/detection3darray.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

namespace apollo
{
namespace safety_layer
{
class DecisionComponent final : public cyber::TimerComponent
{
public:

	DecisionComponent();

	~DecisionComponent();

	bool
	Init() override;

	bool
	Proc() override;

private:

	void
	ProcessDepthClusteringDetection(
			const std::shared_ptr<common::Detection3DArray> depth_clustering_detection_message);

	std::shared_ptr<cyber::Reader<common::Detection3DArray>> depth_clustering_detection_reader_;
	std::shared_ptr<cyber::Writer<control::ControlCommand>> control_command_writer_;
};

CYBER_REGISTER_COMPONENT (DecisionComponent)
} // safety_layer
} // apollo
