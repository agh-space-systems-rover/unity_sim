#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <unity_rs_publisher_msgs/msg/camera_metadata.hpp>

namespace unity_rs_publisher {

class UnityRsPublisher : public rclcpp::Node {
public:
	// ROS
	image_transport::ImageTransport it;

	// Variables
	int         server_socket;
	std::mutex  ros_mutex; // Does not sync with executor, but should be enough
	bool        server_running;
	std::thread server_thread;
	std::vector<std::thread> client_threads;

	UnityRsPublisher(const rclcpp::NodeOptions &options)
	    : Node("unity_rs_publisher", options),
	      it(std::shared_ptr<UnityRsPublisher>(this, [](auto *) {})) {
		// Start the server thread
		server_running = true;
		server_thread =
		    std::thread(&UnityRsPublisher::server_thread_handler, this);
	}

	~UnityRsPublisher() {
		// Close the socket
		close(server_socket);
		// Join threads
		server_running = false;
		server_thread.join();
		for (auto &client_thread : client_threads) {
			client_thread.join();
		}
	}

	void server_thread_handler() {
		// Create a socket
		server_socket = socket(AF_UNIX, SOCK_STREAM, 0);
		if (server_socket == -1) {
			RCLCPP_ERROR(get_logger(), "Socket creation failed");
			rclcpp::shutdown();
			throw std::runtime_error("Socket creation failed");
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
		        server_socket,
		        (struct sockaddr *)&server_addr,
		        sizeof(server_addr)
		    ) == -1) {
			RCLCPP_ERROR(get_logger(), "Socket binding failed");
			close(server_socket);
			rclcpp::shutdown();
			throw std::runtime_error("Socket binding failed");
		}

		// Listen for incoming connections
		if (listen(server_socket, 5) == -1) {
			RCLCPP_ERROR(get_logger(), "Socket listen failed");
			close(server_socket);
			rclcpp::shutdown();
			throw std::runtime_error("Socket listen failed");
		}

		RCLCPP_INFO(get_logger(), "Server is listening for connections...");

		// Disable blocking accept()
		int flags = fcntl(server_socket, F_GETFL, 0);
		fcntl(server_socket, F_SETFL, flags | O_NONBLOCK);

		while (server_running) {
			struct sockaddr_un client_addr;
			socklen_t          client_addr_len = sizeof(client_addr);

			// Accept a new connection
			int client_socket = accept(
			    server_socket, (struct sockaddr *)&client_addr, &client_addr_len
			);
			// Handle non-blocking status
			if (client_socket == -1 &&
			    (errno == EAGAIN || errno == EWOULDBLOCK)) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
			    &UnityRsPublisher::client_thread_handler,
			    this,
			    client_socket,
			    client_id,
			    data_size
			);
		}
	}

	void client_thread_handler(
	    int client_socket, std::string client_id, size_t data_size
	) {
		// Create publishers for this client.
		rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
		    aligned_depth_to_color_info_pub,
		    color_info_pub;
		image_transport::Publisher aligned_depth_to_color_pub, color_pub;
		rclcpp::Subscription<
		    unity_rs_publisher_msgs::msg::CameraMetadata>::SharedPtr meta_sub;
		std::optional<unity_rs_publisher_msgs::msg::CameraMetadata>  meta;
		{
			std::unique_lock ros_lock(ros_mutex);

			RCLCPP_INFO(
			    get_logger(),
			    "Client %s connected. Frame data size: %d",
			    client_id.c_str(),
			    static_cast<int>(data_size)
			);

			// Create a publishers for this client.
			aligned_depth_to_color_info_pub =
			    create_publisher<sensor_msgs::msg::CameraInfo>(
			        client_id + "/aligned_depth_to_color/camera_info", 1
			    );
			aligned_depth_to_color_pub = it.advertise(
			    client_id + "/aligned_depth_to_color/image_raw", 1
			);
			color_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
			    client_id + "/color/camera_info", 1
			);
			color_pub = it.advertise(client_id + "/color/image_raw", 1);

			// Subscribe to Unity camera metadata to know the dimensions and
			// depth bounds.
			meta_sub = create_subscription<
			    unity_rs_publisher_msgs::msg::CameraMetadata>(
			    "/" + client_id + "/unity_rs_publisher/meta",
			    1,
			    [&](const unity_rs_publisher_msgs::msg::CameraMetadata::
			            SharedPtr msg) {
				    meta = *msg;
				    meta_sub.reset();
			    }
			);
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

		std::chrono::steady_clock::time_point now =
		    std::chrono::steady_clock::now();
		float  frame_time_smooth        = 0;
		float  frame_time_log_countdown = 5;
		size_t data_received            = 0;
		while (server_running) {
			// Receive data from the client
			int bytes_received = recv(
			    client_socket,
			    data.data() + data_received,
			    data_size - data_received,
			    0
			);
			if (bytes_received <= 0) {
				// Client disconnected or an error occurred
				std::unique_lock ros_lock(ros_mutex);
				RCLCPP_INFO(
				    get_logger(), "Client %s disconnected.", client_id.c_str()
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
				    height,
				    width,
				    CV_8UC4,
				    const_cast<uint8_t *>(data.data() + 8)
				);
				// +8 to skip timestamp at the start of the frame buffer

				// Split RGBA into color and depth using OpenCV
				cv::cvtColor(rgba_image, color_mat, cv::COLOR_RGBA2RGB);
				cv::extractChannel(rgba_image, depth_image_8bit, 3);

				// Convert depth to 16-bit
				depth_image_8bit.convertTo(
				    depth_mat,
				    CV_16UC1,
				    1000.0F / 255.0F * (depth_max - depth_min)
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

				// Fake artifacts in the depth image.
				for (int _ = 0; _ < 100; _++) {
					int artifact_size = rand() % 20 + 1;
					int x = rand() % (depth_mat.cols - artifact_size);
					int y = rand() % (depth_mat.rows - artifact_size);
					depth_mat(cv::Rect(x, y, artifact_size, artifact_size)) = 0;
				}

				// Initialize the rest of color and depth messages.
				color.header.frame_id = client_id + "_color_optical_frame";
				depth.header.frame_id =
				    client_id + "_color_optical_frame"; // aligned to color
				color.width = depth.width = width;
				color.height = depth.height = height;
				color.step     = width * 3; // 3 channels * 8-bit depth
				depth.step     = width * 2; // 1 channel * 16-bit depth
				color.encoding = sensor_msgs::image_encodings::RGB8;
				depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

				// Compose camera info message.
				camera_info.header.frame_id =
				    client_id + "_color_optical_frame";
				camera_info.height           = height;
				camera_info.width            = width;
				camera_info.distortion_model = "plumb_bob";
				camera_info.d                = {0, 0, 0, 0, 0};
				camera_info.k                = {
                    meta->fx, 0, meta->cx, 0, meta->fy, meta->cy, 0, 0, 1
                };
				camera_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
				camera_info.p = {
				    meta->fx,
				    0,
				    meta->cx,
				    0,
				    0,
				    meta->fy,
				    meta->cy,
				    0,
				    0,
				    0,
				    1,
				    0
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

					color.header.stamp       = stamp;
					depth.header.stamp       = stamp;
					camera_info.header.stamp = stamp;

					aligned_depth_to_color_info_pub->publish(camera_info);
					aligned_depth_to_color_pub.publish(depth);
					color_info_pub->publish(camera_info);
					color_pub.publish(color);
				}

				// Measure frame time.
				float frame_time =
				    std::chrono::duration_cast<std::chrono::milliseconds>(
				        std::chrono::steady_clock::now() - now
				    )
				        .count() /
				    1000.0F;
				now               = std::chrono::steady_clock::now();
				frame_time_smooth = frame_time_smooth * 0.9 + frame_time * 0.1;
				// Log frame time.
				frame_time_log_countdown -= frame_time;
				if (frame_time_log_countdown <= 0) {
					std::unique_lock ros_lock(ros_mutex);
					RCLCPP_DEBUG(
					    get_logger(),
					    "Client %s frame time : %d ms ",
					    client_id.c_str(),
					    static_cast<int>(frame_time_smooth * 1000)
					);
					frame_time_log_countdown = 5;
				}
			}
		}
	}
};

} // namespace unity_rs_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(unity_rs_publisher::UnityRsPublisher)
