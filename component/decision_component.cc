#include <assert.h>
#include <math.h>
#include "modules/safety_layer/component/decision_component.h"

namespace apollo
{
namespace safety_layer
{
DecisionComponent::DecisionComponent() :
	reader_chassis_(nullptr), reader_control_command_(nullptr),
	reader_detections_mission_(nullptr), reader_detections_safety_(nullptr),
	reader_mission_layer_trajectory_(nullptr),
	writer_control_command_(nullptr), channel_name_reader_chassis_(
	"/apollo/canbus/chassis"), channel_name_reader_control_command_(
	"/apollo/control"), channel_name_reader_detections_mission_(
	"/apollo/perception/obstacles"), channel_name_reader_detections_safety_(
	"/apollo/safety_layer/lidar/depth_clustering/detections"), channel_name_writer_control_command_(
	"/apollo/safety_layer/decision/control"), depth_clustering_config_file_name_(
	"/apollo/modules/safety_layer/conf/depth_clustering.json"),
	log_directory_name_verifiable_obstacle_detection_(
    "/apollo/data/log/safety_layer.verifiable_obstacle_detection.log.d"),
	localization_position_(0, 0, 0), velocity_(0, 0, 0), accel_(0, 0, 0),
	override_(false), braking_acceleration_(7.0),
	chassis_speed_mps_(0), control_command_brake_(100), control_latency_(0.01),
	coverage_limit_(0.75)
{
	// https://gitlab.engr.illinois.edu/rtesl/synergistic_redundancy/simulation/apollo/-/blob/main/modules/calibration/data/Lincoln2017MKZ/velodyne_params/velodyne128_novatel_extrinsics.yaml
	translation_vector_lidar_.x() = -0.980728924274446;
	translation_vector_lidar_.y() = 0.2579201;
}

DecisionComponent::~DecisionComponent()
{
}

bool
DecisionComponent::Init()
{
	reader_chassis_ = node_->CreateReader<canbus::Chassis>(
		channel_name_reader_chassis_);

    if (!reader_chassis_)
	{
		AERROR << "Failed to create chassis reader.";
		return false; // Sensor input, depend and use
	}

	reader_gps_ = node_->CreateReader<localization::Gps>(
		"/apollo/sensor/gnss/odometry");
	if (!reader_gps_)
	{
		AWARN << "Failed to create GPS reader.";
	}

	reader_control_command_ = node_->CreateReader<control::ControlCommand>(
		channel_name_reader_control_command_);

	if (!reader_control_command_)
	{
		AWARN << "Failed to create control command reader.";
	}

	reader_detections_mission_ = node_->CreateReader<perception::PerceptionObstacles>(
	channel_name_reader_detections_mission_);

	if (!reader_detections_mission_)
	{
		AWARN << "Failed to create mission detections reader.";
	}

	reader_detections_safety_ = node_->CreateReader<perception::PerceptionObstacles>(
		channel_name_reader_detections_safety_);

	if (!reader_detections_safety_)
	{
		AWARN << "Failed to create safety detections reader.";
	}

	writer_control_command_ = node_->CreateWriter<control::ControlCommand>(
		channel_name_writer_control_command_);

	if (!writer_control_command_)
	{
		AWARN << "Failed to create control command writer.";
	}

	verifiable_obstacle_detection_ = std::make_shared<verifiable_obstacle_detection::VerifiableObstacleDetection>();

	if (verifiable_obstacle_detection_)
	{
		if (!verifiable_obstacle_detection_->initializeForApollo(log_directory_name_verifiable_obstacle_detection_))
		{
			AERROR << "Failed to initialize Verifiable Obstacle Detection.";
			return false;
		}

		if (FLAGS_minloglevel > 0)
		{
			verifiable_obstacle_detection_->disableConsoleLogging();
		}
	}
	else
	{
		AWARN << "Failed to create Verifiable Obstacle Detection.";
	}

	reader_mission_layer_trajectory_ = node_->CreateReader<planning::ADCTrajectory>(
		"/apollo/planning");
	if (!reader_mission_layer_trajectory_)
	{
		AWARN << "Failed to create trajectory reader.";
	}

	return true;
}

bool
DecisionComponent::Proc()
{
	if (reader_chassis_)
    {
		reader_chassis_->Observe();
		ProcessChassis(reader_chassis_->GetLatestObserved());
    }
    else
	{
		AERROR << "Chassis reader missing.";
	}

	if (reader_gps_)
	{
		reader_gps_->Observe();
		ProcessGPS(reader_gps_->GetLatestObserved());
	}
    else
	{
		AERROR << "GPS reader missing.";
	}


	if (reader_detections_mission_)
    {
		reader_detections_mission_->Observe();

		if (reader_detections_safety_)
		{
			reader_detections_safety_->Observe();
			ProcessDetections(reader_detections_mission_->GetLatestObserved(),
				reader_detections_safety_->GetLatestObserved());

			if (reader_mission_layer_trajectory_)
			{
				reader_mission_layer_trajectory_->Observe();
				override_ = OverrideDecision(
							verifiable_obstacle_detection_->getDetectionsSafetyDistanceEndPoints(),
							verifiable_obstacle_detection_->getDetectionsSafetyCoverages(),
							reader_mission_layer_trajectory_->GetLatestObserved());
			}
			else
			{
				AWARN << "Mission trajectory reader missing.";
			}
		}
		else
		{
			AWARN << "Safety detections reader missing.";
		}
    }
    else
	{
		AWARN << "Mission detections reader missing.";
	}

	if (reader_control_command_)
    {
		reader_control_command_->Observe();
		ProcessControlCommand(reader_control_command_->GetLatestObserved());
    }
    else
	{
		AWARN << "Control command reader missing.";
	}

	return true;
}

void
DecisionComponent::ProcessChassis(const std::shared_ptr<canbus::Chassis> chassis)
{
	if (!chassis)
	{
		AERROR << "Chassis missing.";
        return;
	}

	AINFO << "Processing chassis.";
	chassis_speed_mps_ = chassis->speed_mps();
}

void
DecisionComponent::ProcessGPS(const std::shared_ptr<localization::Gps> gps_message)
{
	if (!gps_message)
	{
		AERROR << "GPS missing.";
        return;
	}

	auto position = gps_message->localization().position();
	auto velocity = gps_message->localization().linear_velocity();
	auto accel = gps_message->localization().linear_acceleration();

	localization_position_.x() = position.x();
	localization_position_.y() = position.y();
	localization_position_.z() = position.z();

	// The negative sign makes obstacle position and velocity vectors signs align
	velocity_.x() = -velocity.x(); 
	velocity_.y() = -velocity.y();
	velocity_.z() = -velocity.z();

	accel_.x() = -accel.x(); 
	accel_.y() = -accel.y();
	accel_.z() = -accel.z();
}


void
DecisionComponent::ProcessControlCommand(const
	std::shared_ptr<control::ControlCommand> control_command)
{
	if (!control_command)
	{
		AERROR << "Control command missing.";
        return;
	}

	if (!writer_control_command_)
	{
		AERROR << "Control command writer missing.";
        return;
	}

	AINFO << "Processing control command.";

	auto control_command_override = std::make_shared<control::ControlCommand>(*control_command);

	if (override_)
	{
		// control_command_override->set_steering_rate(0);
		// control_command_override->set_steering_target(0);
		control_command_override->set_speed(0);
		control_command_override->set_throttle(0);
		control_command_override->set_brake(control_command_brake_);
		AINFO << "Activated safety override.";
	}

	writer_control_command_->Write(control_command_override);
}

void
DecisionComponent::ProcessDetections(
	const std::shared_ptr<perception::PerceptionObstacles> detections_mission,
	const std::shared_ptr<perception::PerceptionObstacles> detections_safety)
{
	if (!detections_mission)
	{
		AERROR << "Mission detections missing.";
        return;
	}

	if (!detections_safety)
	{
		AERROR << "Safety detections missing.";
        return;
	}

	AINFO << "Processing detections.";

	const std::string sequence_num = std::to_string(detections_mission->header().sequence_num());
	const std::string frame_name = sequence_num + ".bin";
	auto depth_clustering_parameter_factory = depth_clustering::ParameterFactory(
		depth_clustering_config_file_name_);
	const auto& depth_clustering_parameter = depth_clustering_parameter_factory.getDepthClusteringParameter();
	const auto& bounding_box_type = depth_clustering_parameter.bounding_box_type;

	verifiable_obstacle_detection_->processOneFrameForApollo(frame_name,
		convertMissionDetectionsToPolygons(detections_mission),
		convertDetectionsToPolygons(detections_safety, bounding_box_type));

}

double
DecisionComponent::DistanceAdjust(double distance)
{
	return (distance * 0.95) - 0.1; // safety margin as per vOD paper, here we deduct it from distance to make conservative decisions.
}

double
DecisionComponent::DistanceFromPoints(const verifiable_obstacle_detection::Point2D a, const verifiable_obstacle_detection::Point2D b)
{
	return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2));
}

bool
DecisionComponent::OverrideDecision(
	const std::vector<std::pair<verifiable_obstacle_detection::Point2D, verifiable_obstacle_detection::Point2D>> safety_closest_points,
	const std::vector<double> overlaps, const std::shared_ptr<planning::ADCTrajectory> mission_layer_trajectory_message)
{
	if (!mission_layer_trajectory_message)
	{
		AERROR << "Mission Layer Trajectory Missing.";
        return false; // TODO should be true but simulation setup is finicky 
	}

	Eigen::Vector3d ego_extent;
	ego_extent.x() = 5.02;
	ego_extent.y() = 2.13;
	ego_extent.z() = 1.50;	
	double braking_distance;
	braking_distance = (chassis_speed_mps_ * control_latency_); 	// override decision to start distance // TODO: get control latency
	braking_distance += ((chassis_speed_mps_ * chassis_speed_mps_) / (2 * braking_acceleration_)); // Deceleration distance
	double time_to_stop = chassis_speed_mps_ / braking_acceleration_ + control_latency_;

	// Only for fault injection scenario, return override value, rather than resetting it
	// It supresses the fact that mission layer actually sees this obstacle and thus
	// is able to plan around it.
	// if (safety_closest_points.size() > 0)
	// {
	// 	return true;
	// }

	assert (safety_closest_points.size() == overlaps.size());

	// TODO: This assumes scenario information, make it generic for future works.
	for (unsigned int i = 0; i < safety_closest_points.size(); i++)
	{
		const std::pair<verifiable_obstacle_detection::Point2D, verifiable_obstacle_detection::Point2D> closest_points = safety_closest_points[i];
		const double overlap = overlaps[i];

		if (overlap >= coverage_limit_)
		{
			// Since the obstacle is detected by mission layer, skip this obstacle
			// Disable this for FN Fault injection, but mission layer path updates will muddle things.
			continue;
		}

		// double distance_from_lidar = DistanceAdjust(DistanceFromPoints(closest_points.second, verifiable_obstacle_detection::Point2D(0, 0)));
		double distance_between_points = DistanceAdjust(DistanceFromPoints(closest_points.second, closest_points.first));

		// Optimization
		// Since closest separation is large enough,
		// no need to do trajectory based finer analysis.
		if (distance_between_points > braking_distance)
		{
			continue;
		}

		// Assumes trajectory upto time to stop is available.		
		for (const auto& trajectory_point : mission_layer_trajectory_message->trajectory_point())
		{
			if (trajectory_point.relative_time() < 0 || trajectory_point.relative_time() > time_to_stop)
			{
				continue;
			}

			double path_point_x = trajectory_point.path_point().y() - localization_position_.y();
			double path_point_y = trajectory_point.path_point().x() - localization_position_.x();
			verifiable_obstacle_detection::Point2D moved_closest_point_av = verifiable_obstacle_detection::Point2D(
													closest_points.first.x() + path_point_x, closest_points.first.y() + path_point_y);
			double distance_closest_from_lidar_traj = DistanceAdjust(DistanceFromPoints(closest_points.second, moved_closest_point_av));
				// sqrt(
				// pow(closest_points.second.x() - (closest_points.first.x() + path_point_x), 2) +
				// pow(closest_points.second.y() - (closest_points.first.y() + path_point_y), 2)));

			// We are assuming here that closest points don't change over the trajectory
			// Ideally we should recalculate the closest points for each step in trajectory.
			// Here we are comparing the distance to 10 cm, but that's only because obstacle is stationary.
			// Otherwise we would be comparing it to the obstacle existence region radius.
			if (distance_closest_from_lidar_traj < 0.1)
			{
				return true;
			}

			// double vel  = trajectory_point.v();

			// // Assumes constant heading
			// double v_x = velocity_.x() * (vel / speed_mps_);
			// double v_y = velocity_.y() * (vel / speed_mps_);

			// double b   = pow(vel, 2) / (2 * braking_acceleration_) + (vel * braking_slack_time_);
			// double b_x = b * (v_x / vel);
			// double b_y = b * (v_y / vel);

			// double d_x = x_o - path_point_x;
			// double d_y = y_o - path_point_y;

			// // Very close to the ego vehicle DC is unable to detect properly due to algorithmic issues
			// // TODO: Need to remove this after tracking is working
			// if ((d_x < d_crit_x) && (d_y < d_crit_y))
			// {
			// 	continue;
			// }
		}
	}



	// Normal return
	return false;


}


std::vector<verifiable_obstacle_detection::Polygon>
DecisionComponent::convertDetectionsToPolygons(const std::shared_ptr<perception::PerceptionObstacles> detections,
	depth_clustering::BoundingBox::Type bounding_box_type)
{
	std::vector<verifiable_obstacle_detection::Polygon> polygons;

	switch (bounding_box_type)
	{
	default:
	{
		AWARN << "Unknown bounding box type " << static_cast<int>(bounding_box_type) << ".";
	}
	case depth_clustering::BoundingBox::Type::Cube:
	{
		AINFO << "Converting cube detections to polygons.";
		AWARN << "Polygon conversion for cube detections not implemented.";
		break;
	}
	case depth_clustering::BoundingBox::Type::Polygon:
	{
		AINFO << "Converting polygon detections to polygons.";

		for (const auto& perception_obstacle : detections->perception_obstacle())
		{
			verifiable_obstacle_detection::Polygon polygon;
			std::vector<verifiable_obstacle_detection::Point2D> polygon_points;

			for (const auto& polygon_point : perception_obstacle.polygon_point())
			{
				polygon_points.push_back(verifiable_obstacle_detection::Point2D(polygon_point.x(), polygon_point.y()));
			}

			boost::geometry::assign_points(polygon, polygon_points);
			boost::geometry::correct(polygon);

			polygons.push_back(polygon);
		}

		break;
	}
	case depth_clustering::BoundingBox::Type::Flat:
	{
		AINFO << "Converting flat detections to polygons.";
		AWARN << "Polygon conversion for flat detections not implemented.";
		break;
	}
	}

	return polygons;
}

std::vector<verifiable_obstacle_detection::Polygon>
DecisionComponent::convertMissionDetectionsToPolygons(const std::shared_ptr<perception::PerceptionObstacles> detections)
{
	std::vector<verifiable_obstacle_detection::Polygon> polygons;

	AINFO << "Converting mission detections to polygons.";

	for (const auto& perception_obstacle : detections->perception_obstacle())
	{
		verifiable_obstacle_detection::Polygon polygon;
		std::vector<verifiable_obstacle_detection::Point2D> polygon_points;

		for (const auto& polygon_point : perception_obstacle.polygon_point())
		{
			const auto polygon_point_x = translation_vector_lidar_.x() + polygon_point.y() - localization_position_.y();
			const auto polygon_point_y = translation_vector_lidar_.y() - (polygon_point.x() - localization_position_.x());

			polygon_points.push_back(verifiable_obstacle_detection::Point2D(polygon_point_x, polygon_point_y));
		}

		boost::geometry::assign_points(polygon, polygon_points);
		boost::geometry::correct(polygon);

		polygons.push_back(polygon);
	}

	return polygons;
}


}	/* safety_layer */
}	/* apollo */
