#include <string>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

struct Position2D{
  double x;
  double y;
};

// Calculate Euclidean distance between two positions
double CalcDistance(const Position2D& p1, const Position2D& p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

class MarkerHandler {
public:
  MarkerHandler() = default;

  void Init(ros::NodeHandle& nh) {
    pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  }

  bool IsReady() {
    return pub_.getNumSubscribers() > 0;
  }

  void ShowMarker(const Position2D& pos, int id) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = kMarkerNs;
    marker.id = id;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    pub_.publish(marker);
  }

  void HideMarker(int id) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = kMarkerNs;
    marker.id = id;
    marker.action = visualization_msgs::Marker::DELETE;

    pub_.publish(marker);
  }

private:
  const std::string kMarkerNs = "add_markers";
  ros::Publisher pub_;
};

class AddMarkersNode {
public:
  AddMarkersNode(const Position2D& pickup_pos, const Position2D& dropoff_pos):
    pickup_pos_(pickup_pos),
    dropoff_pos_(dropoff_pos) {};

  void Run() {
    ros::NodeHandle private_nh("~");

    // Initialize MarkerHandler
    marker_handler_.Init(nh_);

    // Wait for marker subscribers
    while (!marker_handler_.IsReady()) {
      ROS_INFO("Waiting subscribers");
      if (!ros::ok()) {
	return;
      }
      sleep(1);
    }

    // Get parameter
    bool test_run;
    private_nh.param("test_run", test_run, false);

    if (test_run) {
      RunForTesting();
    } else {
      RunForHomeServiceRobot();
    }
  }

private:
  /*
   * Show pickup/dropoff markers alternately for testing.
   * This function repeats:
   * 1. show pickup marker for 5 seconds
   * 2. pause 5 sec
   * 3. show dropoff marker for 5 seconds
   * 4. pause 5 sec
   */
  void RunForTesting() {
    while (ros::ok()) {
      // Show pickup marker for 5 sec
      ROS_INFO("Show pickup marker");
      marker_handler_.ShowMarker(pickup_pos_, kPickupMarkerId);

      ROS_INFO("Wait 5 sec");
      ros::Duration(5.0).sleep();

      ROS_INFO("Hide pickup marker");
      marker_handler_.HideMarker(kPickupMarkerId);

      // Pause 5 sec
      ROS_INFO("Wait 5 sec");
      ros::Duration(5.0).sleep();

      // Show dropoff marker for 5 sec
      ROS_INFO("Show dropoff marker");
      marker_handler_.ShowMarker(dropoff_pos_, kDropoffMarkerId);

      ROS_INFO("Wait 5 sec");
      ros::Duration(5.0).sleep();

      ROS_INFO("Hide dropoff marker");
      marker_handler_.HideMarker(kDropoffMarkerId);

      // Pause 5 sec
      ROS_INFO("Wait 5 sec");
      ros::Duration(5.0).sleep();
    }
  }

  /*
   * Show markers for Home Service Robot.
   * 1. Initially show the pickup marker
   * 2. Hide the pickup marker once the robot reaches the pickup zone
   * 3. Show the dropoff marker once the robot reaches the dropoff zone
   */
  void RunForHomeServiceRobot() {
    // Initially show pickup marker
    ROS_INFO("Show pickup marker");
    marker_handler_.ShowMarker(pickup_pos_, kPickupMarkerId);
    reached_pickup_ = false;
    reached_dropoff_ = false;

    // Subscribe AMCL estimated pose
    ROS_INFO("Subscribe amcl_pose");
    sub_ = nh_.subscribe("amcl_pose", 10, &AddMarkersNode::HandleMessage, this);

    ros::spin();
  }
  
  void HandleMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    Position2D robot_pos{ msg->pose.pose.position.x, msg->pose.pose.position.y };

    // Use Euclidean distance to check if the robot has reached pickup/dropoff zone.
    const double kThresholdDistance = 0.5;

    if (!reached_pickup_) {
      double dist_pickup = CalcDistance(robot_pos, pickup_pos_);

      if (dist_pickup < kThresholdDistance) {
	// The robot has reached the pickup zone
	ROS_INFO("Hide pickup marker");
	marker_handler_.HideMarker(kPickupMarkerId);
	reached_pickup_ = true;
      }       
    } else if (!reached_dropoff_) {
      double dist_dropoff = CalcDistance(robot_pos, dropoff_pos_);

      if (dist_dropoff < kThresholdDistance) {
	// The robot has reached the dropoff zone
	ROS_INFO("Show dropoff marker");
	marker_handler_.ShowMarker(dropoff_pos_, kDropoffMarkerId);
	reached_dropoff_ = true;
      }
    }
  }

  const Position2D pickup_pos_;
  const Position2D dropoff_pos_;
  const int kPickupMarkerId = 1;
  const int kDropoffMarkerId = 2;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  MarkerHandler marker_handler_;
  bool reached_pickup_;
  bool reached_dropoff_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");

  ros::NodeHandle nh, private_nh("~");

  const Position2D kPickupPos{ 6.9, 0.4 };
  const Position2D kDropoffPos{ -4.2, -2.2 };

  AddMarkersNode node(kPickupPos, kDropoffPos);

  node.Run();

  return 0;
}
