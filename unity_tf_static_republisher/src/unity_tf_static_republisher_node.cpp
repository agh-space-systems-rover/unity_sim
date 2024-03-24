// The UnityTFStaticRepublisher class is a simple ROS2 node that subscribes to
// the /tf_static topic and republishes the received messages on a new topic.
// The  tf_static message is stored in a member variable and republished at a
// fixed rate. This mechanism is needed because Unity uses rosbridge_suite to
// communicate with ROS2, and rosbridge_suite won't deterministically receive
// the /tf_static messages. This node is a workaround to ensure that Unity can
// receive /tf_static messages properly.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

constexpr const char *pub_topic = "tf_static/republished_for_unity";

constexpr int    default_queue_size = 10;
constexpr double default_rate       = 1.0;

namespace unity_tf_static_republisher {

class UnityTFStaticRepublisher : public rclcpp::Node {
  public:
	UnityTFStaticRepublisher(const rclcpp::NodeOptions &options)
	    : Node("unity_tf_static_republisher", options) {
		// Declare parameters.
		declare_parameter("queue_size", default_queue_size);
		declare_parameter("rate", default_rate);

		// Read static parameters.
		int    queue_size = default_queue_size;
		double rate       = default_rate;
		get_parameter("queue_size", queue_size);
		get_parameter("rate", rate);

		// Subscribe to the tf_static topic
		// Adapted from
		// https://github.com/ros2/geometry2/blob/humble/tf2_ros/include/tf2_ros/transform_listener.h
		rclcpp::QoS qos(100);
		qos.transient_local();
		rclcpp::SubscriptionOptions sub_options;
		sub_options.qos_overriding_options = rclcpp::QosOverridingOptions{
		    rclcpp::QosPolicyKind::Depth,
		    rclcpp::QosPolicyKind::History,
		    rclcpp::QosPolicyKind::Reliability
		};
		sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
		    "tf_static",
		    qos,
		    std::bind(
		        &UnityTFStaticRepublisher::tfStaticCallback,
		        this,
		        std::placeholders::_1
		    ),
		    sub_options
		);

		// Publish the recorded tf_static messages on a chosen topic
		pub = this->create_publisher<tf2_msgs::msg::TFMessage>(
		    pub_topic, queue_size
		);

		timer = this->create_wall_timer(
		    std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
		    std::bind(&UnityTFStaticRepublisher::republishTFStatic, this)
		);
	}

  private:
	void tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
		RCLCPP_INFO(
		    this->get_logger(),
		    "Received tf_static. Republishing on %s",
		    pub_topic
		);
		// Store the transforms in a map, indexed by the frame_id and
		// child_frame_id
		for (auto &transform : msg->transforms) {
			last_transforms[std::make_pair(
			    transform.header.frame_id, transform.child_frame_id
			)] = transform;
		}
	}

	void republishTFStatic() {
		if (!last_transforms.empty()) {
			// Construct a new message with the stored transforms.
			tf2_msgs::msg::TFMessage msg;
			for (auto &pair : last_transforms) {
				msg.transforms.push_back(pair.second);
			}
			pub->publish(msg);
		}
	}

	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr    pub;
	rclcpp::TimerBase::SharedPtr                              timer;
	std::map<
	    std::pair<std::string, std::string>,
	    geometry_msgs::msg::TransformStamped>
	    last_transforms;
};

} // namespace unity_tf_static_republisher

RCLCPP_COMPONENTS_REGISTER_NODE(
    unity_tf_static_republisher::UnityTFStaticRepublisher
)
