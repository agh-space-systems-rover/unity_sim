#ifdef ROS_HUMBLE
	#include <cv_bridge/cv_bridge.h>
#else
	#include <cv_bridge/cv_bridge.hpp>
#endif
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <csignal>
#include <cstring>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <unity_rs_publisher_msgs/msg/camera_metadata.hpp>
#include <vector>

constexpr size_t point_cloud_height      = 144;
constexpr float  frame_time_smoothing    = 0.9;
constexpr float  frame_time_log_interval = 5;

std::mutex                                       ros_mutex;
rclcpp::Node::SharedPtr                          ros_node;
std::shared_ptr<image_transport::ImageTransport> ros_it;

// Function to handle client messages
void handle_client(int client_socket, std::string client_id, size_t data_size) {
	// Create publishers for this client.
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
	    aligned_depth_to_color_info_pub,
	    color_info_pub;
	image_transport::Publisher aligned_depth_to_color_pub, color_pub;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
	rclcpp::Subscription<
	    unity_rs_publisher_msgs::msg::CameraMetadata>::SharedPtr meta_sub;
	std::optional<unity_rs_publisher_msgs::msg::CameraMetadata>  meta;
	{
		std::unique_lock ros_lock(ros_mutex);

		RCLCPP_INFO(
		    ros_node->get_logger(),
		    "Client %s connected. Frame data size: %d",
		    client_id.c_str(),
		    static_cast<int>(data_size)
		);

		// Create a publishers for this client.
		// See: https://github.com/IntelRealSense/realsense-ros
		//
		aligned_depth_to_color_info_pub =
		    ros_node->create_publisher<sensor_msgs::msg::CameraInfo>(
		        "/" + client_id + "/aligned_depth_to_color/camera_info", 1
		    );
		aligned_depth_to_color_pub = ros_it->advertise(
		    "/" + client_id + "/aligned_depth_to_color/image_raw", 1
		);
		color_info_pub =
		    ros_node->create_publisher<sensor_msgs::msg::CameraInfo>(
		        "/" + client_id + "/color/camera_info", 1
		    );
		color_pub = ros_it->advertise("/" + client_id + "/color/image_raw", 1);
		// - no color metadata
		// - depth info same as color
		// points_pub =
		// ros_node->create_publisher<sensor_msgs::msg::PointCloud2>(
		//     "/" + client_id + "/depth/color/points", 1
		// );
		// NOTE: point clouds were disabled because we can generate them in ROS
		// Also, actual RealSense PCs are very high detail, so a custom cloud
		// pipeline is needed.
		// - no depth camera_info
		// - no depth images
		// - no depth metadata
		// - no depth_to_color extrinsics
		// - no diagnostics, parameter_events, rosout, tf_static, etc.

		// Subscribe to Unity camera metadata to know the dimensions and depth
		// bounds.
		meta_sub = ros_node->create_subscription<
		    unity_rs_publisher_msgs::msg::CameraMetadata>(
		    "/" + client_id + "/unity_rs_publisher/meta",
		    1,
		    [&meta](
		        const unity_rs_publisher_msgs::msg::CameraMetadata::SharedPtr
		            msg
		    ) {
			    meta = *msg;
		    }
		);

		rclcpp::spin_some(ros_node);
	}

	// persistent buffers
	std::vector<uint8_t>          data(data_size);
	sensor_msgs::msg::Image       color, depth;
	cv::Mat                       depth_image_8bit;
	sensor_msgs::msg::PointCloud2 point_cloud;
	sensor_msgs::msg::CameraInfo  camera_info;

	// RNG for point cloud subsampling
	std::mt19937                          rng{std::random_device{}()};
	std::uniform_real_distribution<float> dist{-1, 1};

	// std::chrono::steady_clock::time_point now =
	// std::chrono::steady_clock::now(); float frame_time_smooth = 0; float
	// frame_time_log_countdown = frame_time_log_interval;
	size_t data_received = 0;
	while (true) {
		// Receive data from the client
		int bytes_received = recv(
		    client_socket,
		    data.data() + data_received,
		    data_size - data_received,
		    0
		);
		if (bytes_received <= 0) {
			// Client disconnected or an error occurred
			RCLCPP_INFO(
			    ros_node->get_logger(),
			    "Client %s disconnected.",
			    client_id.c_str()
			);
			close(client_socket);

			break;
		}

		// Check if all data has been received
		data_received += bytes_received;
		if (data_received == data_size) {
			// Reset the number of bytes received
			data_received = 0;

			// Skip frame if no camera metadata was yet received.
			if (!meta.has_value()) {
				std::unique_lock ros_lock(ros_mutex);
				rclcpp::spin_some(ros_node);
				continue;
			}

			// Extract properties from camera info.
			uint32_t width     = meta->width;
			uint32_t height    = meta->height;
			float    depth_min = meta->depth_min;
			float    depth_max = meta->depth_max;

			// Extract timestamp from the first 8 bytes of the frame buffer.
			int32_t  sec     = *(int32_t *)data.data();
			uint32_t nanosec = *(uint32_t *)(data.data() + 4);

			// Initialize 8-bit depth CV Mat.
			depth_image_8bit.create(height, width, CV_8UC1);

			// Initialize color and depth images.
			// -8 to account for timestamp at the start of the frame buffer
			color.data.resize(
			    (data.size() - 8) / 4 * 3
			); // 3 channels * 8-bit depth
			depth.data.resize(
			    (data.size() - 8) / 4 * 2
			); // 1 channel * 16-bit depth

			// Create OpenCV wrappers for color and depth data.
			cv::Mat color_mat(height, width, CV_8UC3, color.data.data());
			cv::Mat depth_mat(height, width, CV_16UC1, depth.data.data());

			// Prepare OpenCV Mat for RGBA data
			cv::Mat rgba_image(
			    height, width, CV_8UC4, const_cast<uint8_t *>(data.data() + 8)
			);
			// +8 to skip timestamp at the start of the frame buffer

			// Split RGBA into color and depth using OpenCV
			cv::cvtColor(rgba_image, color_mat, cv::COLOR_RGBA2RGB);
			cv::extractChannel(rgba_image, depth_image_8bit, 3);

			// Convert depth to 16-bit
			depth_image_8bit.convertTo(
			    depth_mat, CV_16UC1, 1000.0F / 255.0F * (depth_max - depth_min)
			);

			// Add depth_min * 1000.0F to all non-zero (valid) pixels.
			cv::add(
			    depth_mat,
			    cv::Scalar(depth_min * 1000.0F),
			    depth_mat,
			    depth_mat != 0
			);

			// Flip both images vertically.
			// OpenGL uses bottom-left origin, while OpenCV uses top-left
			// origin.
			cv::flip(color_mat, color_mat, 0);
			cv::flip(depth_mat, depth_mat, 0);

			// Initialize the rest of color and depth messages.
			color.header.frame_id = client_id + "_color_optical_frame";
			depth.header.frame_id =
			    client_id + "_color_optical_frame"; // aligned to color
			color.width = depth.width = width;
			color.height = depth.height = height;
			color.step                  = width * 3; // 3 channels * 8-bit depth
			depth.step                  = width * 2; // 1 channel * 16-bit depth
			color.encoding              = sensor_msgs::image_encodings::RGB8;
			depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

			// // Generate point cloud.
			// size_t point_cloud_width =
			//     static_cast<float>(width) / height * point_cloud_height;
			// point_cloud.data.reserve(
			//     point_cloud_width * point_cloud_height * 4 * 4
			// ); // 4 bytes per float; 4 floats per point (x, y, z, rgb); rgb
			// is
			//    // actually uint32
			// point_cloud.data.clear();
			// float subsample_radius =
			//     static_cast<float>(height) / point_cloud_height / 2;
			// for (size_t row = 0; row < point_cloud_height; row++) {
			// 	for (size_t col = 0; col < point_cloud_width; col++) {
			// 		size_t src_row = row * (static_cast<float>(height) /
			// 		                        point_cloud_height) +
			// 		                 0.5;
			// 		size_t src_col = col * (static_cast<float>(height) /
			// 		                        point_cloud_height) +
			// 		                 0.5;

			// 		// Subsample points.
			// 		src_row = static_cast<float>(src_row) +
			// 		          dist(rng) * subsample_radius;
			// 		src_col = static_cast<float>(src_col) +
			// 		          dist(rng) * subsample_radius;

			// 		if (src_row >= height || src_col >= width) {
			// 			continue;
			// 		}

			// 		uint8_t src_depth_low =
			// 		    depth.data[(src_row * width + src_col) * 2 + 0];
			// 		uint8_t src_depth_high =
			// 		    depth.data[(src_row * width + src_col) * 2 + 1];
			// 		uint32_t src_depth = static_cast<uint32_t>(src_depth_high)
			// 		                      << 8 |
			// 		                     src_depth_low;
			// 		uint32_t src_r =
			// 		    color.data[(src_row * width + src_col) * 3 + 0];
			// 		uint32_t src_g =
			// 		    color.data[(src_row * width + src_col) * 3 + 1];
			// 		uint32_t src_b =
			// 		    color.data[(src_row * width + src_col) * 3 + 2];

			// 		if (src_depth != 0) {
			// 			size_t i = point_cloud.data.size();
			// 			point_cloud.data.resize(point_cloud.data.size() + 16);

			// 			float    z   = src_depth / 1000.0F;
			// 			float    x   = (src_col - meta->cx) * z / meta->fx;
			// 			float    y   = (src_row - meta->cy) * z / meta->fy;
			// 			uint32_t rgb = (src_r & 0xff) << 16 |
			// 			               (src_g & 0xff) << 8 | (src_b & 0xff);

			// 			// Add a point to the buffer.
			// 			*reinterpret_cast<float *>(&point_cloud.data[i + 0]) =
			// 			    x;
			// 			*reinterpret_cast<float *>(&point_cloud.data[i + 4]) =
			// 			    y;
			// 			*reinterpret_cast<float *>(&point_cloud.data[i + 8]) =
			// 			    z;
			// 			*reinterpret_cast<uint32_t *>(&point_cloud.data[i + 12]
			// 			) = rgb;
			// 		}
			// 	}
			// }

			// // Initialize the rest of the point cloud message.
			// point_cloud.header.frame_id = client_id + "_color_optical_frame";
			// point_cloud.height          = 1;
			// point_cloud.width           = point_cloud.data.size() / 16;
			// point_cloud.fields.resize(4);
			// point_cloud.fields[0].name   = "x";
			// point_cloud.fields[0].offset = 0;
			// point_cloud.fields[0].datatype =
			//     sensor_msgs::msg::PointField::FLOAT32;
			// point_cloud.fields[0].count  = 1;
			// point_cloud.fields[1].name   = "y";
			// point_cloud.fields[1].offset = 4;
			// point_cloud.fields[1].datatype =
			//     sensor_msgs::msg::PointField::FLOAT32;
			// point_cloud.fields[1].count  = 1;
			// point_cloud.fields[2].name   = "z";
			// point_cloud.fields[2].offset = 8;
			// point_cloud.fields[2].datatype =
			//     sensor_msgs::msg::PointField::FLOAT32;
			// point_cloud.fields[2].count  = 1;
			// point_cloud.fields[3].name   = "rgb";
			// point_cloud.fields[3].offset = 12;
			// point_cloud.fields[3].datatype =
			//     sensor_msgs::msg::PointField::UINT32;
			// point_cloud.fields[3].count = 1;
			// point_cloud.is_bigendian    = false;
			// point_cloud.point_step      = 16;
			// point_cloud.row_step        = point_cloud.data.size();
			// point_cloud.is_dense        = true;

			// Compose camera info message.
			camera_info.header.frame_id  = client_id + "_color_optical_frame";
			camera_info.height           = height;
			camera_info.width            = width;
			camera_info.distortion_model = "plumb_bob";
			camera_info.d                = {0, 0, 0, 0, 0};
			camera_info.k                = {
                meta->fx, 0, meta->cx, 0, meta->fy, meta->cy, 0, 0, 1
            };
			camera_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
			camera_info.p = {
			    meta->fx, 0, meta->cx, 0, 0, meta->fy, meta->cy, 0, 0, 0, 1, 0
			};
			camera_info.binning_x      = 0;
			camera_info.binning_y      = 0;
			camera_info.roi.x_offset   = 0;
			camera_info.roi.y_offset   = 0;
			camera_info.roi.height     = 0;
			camera_info.roi.width      = 0;
			camera_info.roi.do_rectify = false;

			// Publish.
			{
				std::unique_lock ros_lock(ros_mutex);

				builtin_interfaces::msg::Time stamp;
				stamp.sec     = sec;
				stamp.nanosec = nanosec;

				color.header.stamp = stamp;
				depth.header.stamp = stamp;
				// point_cloud.header.stamp = stamp;
				camera_info.header.stamp = stamp;

				aligned_depth_to_color_info_pub->publish(camera_info);
				aligned_depth_to_color_pub.publish(depth);
				color_info_pub->publish(camera_info);
				color_pub.publish(color);
				// points_pub->publish(point_cloud);
				rclcpp::spin_some(ros_node);
			}

			// // Measure frame time.
			// float frame_time =
			// std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
			// - now).count() / 1000.0F; now = std::chrono::steady_clock::now();
			// frame_time_smooth = frame_time_smooth * frame_time_smoothing +
			// frame_time * (1 - frame_time_smoothing);

			// // Log frame time.
			// frame_time_log_countdown -= frame_time;
			// if (frame_time_log_countdown <= 0) {
			//     RCLCPP_INFO(ros_node->get_logger(), "Client %s frame time: %d
			//     ms", client_id.c_str(), static_cast<int>(frame_time_smooth *
			//     1000)); frame_time_log_countdown = frame_time_log_interval;
			// }
		}
	}
}

// Signal handler for graceful exit
void signal_handler(int) {
	unlink("/tmp/unity_rs_publisher");
	exit(0);
}

int main(int argc, char **argv) {
	// Create a ROS node
	rclcpp::init(argc, argv);
	ros_node = std::make_shared<rclcpp::Node>("unity_rs_publisher");
	ros_it   = std::make_shared<image_transport::ImageTransport>(ros_node);

	// Create a socket
	int server_socket = socket(AF_UNIX, SOCK_STREAM, 0);
	if (server_socket == -1) {
		RCLCPP_ERROR(ros_node->get_logger(), "Socket creation failed");
		rclcpp::shutdown();
		return 1;
	}

	struct sockaddr_un server_addr;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sun_family = AF_UNIX;
	strncpy(
	    server_addr.sun_path,
	    "/tmp/unity_rs_publisher",
	    sizeof(server_addr.sun_path) - 1
	);

	// Bind the socket to the server address
	unlink("/tmp/unity_rs_publisher"
	); // Make sure the socket doesn't already exist.
	if (bind(
	        server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)
	    ) == -1) {
		RCLCPP_ERROR(ros_node->get_logger(), "Socket binding failed");
		close(server_socket);
		rclcpp::shutdown();
		return 1;
	}

	// Register a signal handler for Ctrl+C (SIGINT) + SIGTERM + SIGQUIT +
	// SIGABRT + SIGHUP + SIGKILL
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGQUIT, signal_handler);
	signal(SIGABRT, signal_handler);
	signal(SIGHUP, signal_handler);
	signal(SIGKILL, signal_handler);

	// Listen for incoming connections
	if (listen(server_socket, 5) == -1) {
		RCLCPP_ERROR(ros_node->get_logger(), "Socket listen failed");
		close(server_socket);
		rclcpp::shutdown();
		return 1;
	}

	RCLCPP_INFO(
	    ros_node->get_logger(), "Server is listening for connections..."
	);

	std::vector<std::thread> client_threads;

	while (true) {
		struct sockaddr_un client_addr;
		socklen_t          client_addr_len = sizeof(client_addr);

		// Accept a new connection
		int client_socket = accept(
		    server_socket, (struct sockaddr *)&client_addr, &client_addr_len
		);
		if (client_socket == -1) {
			std::unique_lock ros_lock(ros_mutex);
			RCLCPP_ERROR(ros_node->get_logger(), "Socket accept failed");
			continue;
		}

		// Receive the buffer size and client ID as the handshake message
		char data_size_plus_client_id_buffer[256];
		int  bytes_received = recv(
            client_socket,
            data_size_plus_client_id_buffer,
            sizeof(data_size_plus_client_id_buffer),
            0
        );
		if (bytes_received <= 0) {
			close(client_socket);
			continue;
		}
		// Decode first 4 bytes as the buffer size
		int         data_size = *(int *)data_size_plus_client_id_buffer;
		std::string client_id(data_size_plus_client_id_buffer + 4);

		// Create a new thread to handle the client
		client_threads.emplace_back(
		    handle_client, client_socket, client_id, data_size
		);
	}

	return 0;
}
