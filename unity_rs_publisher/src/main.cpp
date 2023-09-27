#include <iostream>
#include <cstring>
#include <thread>
#include <vector>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <csignal>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

constexpr size_t point_cloud_height = 72;
constexpr float frame_time_smoothing = 0.9;
constexpr float frame_time_log_interval = 5;

std::mutex ros_mutex;
rclcpp::Node::SharedPtr ros_node;
std::shared_ptr<image_transport::ImageTransport> ros_it;

// Function to handle client messages
void handle_client(int client_socket, std::string client_id, size_t data_size) {
    // Create publishers for this client.
    image_transport::Publisher aligned_depth_to_color_pub, color_pub, depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub;
    std::optional<sensor_msgs::msg::CameraInfo> depth_info;
    {
        std::unique_lock ros_lock(ros_mutex);

        RCLCPP_INFO(ros_node->get_logger(), "Client %s connected. Frame data size: %d", client_id.c_str(), static_cast<int>(data_size));

        // Create a publishers for this client.
        // See: https://github.com/IntelRealSense/realsense-ros
        //
        // - no aligned_depth_to_color info
        // - no aligned_depth_to_color images
        // - color info handled by Unity
        color_pub = ros_it->advertise("/" + client_id + "/color/image_raw", 1);
        // - no color metadata
        // - depth info handled by Unity
        points_pub = ros_node->create_publisher<sensor_msgs::msg::PointCloud2>("/" + client_id + "/depth/color/points", 1);
        depth_pub = ros_it->advertise("/" + client_id + "/depth/image_rect_raw", 1);
        // - no depth metadata
        // - no depth_to_color extrinsics
        // - no IMU; TODO: Handle in Unity?
        // - no diagnostics, parameter_events, rosout, tf_static, etc.

        // Subscribe to depth info to generate point clouds.
        depth_info_sub = ros_node->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/" + client_id + "/depth/camera_info", 10,
            [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                depth_info = *msg;
            }
        );

        rclcpp::spin_some(ros_node);
    }
    
    // persistent buffers
    std::vector<uint8_t> data(data_size);
    sensor_msgs::msg::Image color, depth;
    // std::vector<uint8_t> point_cloud;
    sensor_msgs::msg::PointCloud2 point_cloud;

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    float frame_time_smooth = 0;
    float frame_time_log_countdown = frame_time_log_interval;
    size_t data_received = 0;
    while (true) {
        // Receive data from the client
        int bytes_received = recv(client_socket, data.data() + data_received, data_size - data_received, 0);
        if (bytes_received <= 0) {
            // Client disconnected or an error occurred
            RCLCPP_INFO(ros_node->get_logger(), "Client %s disconnected.", client_id.c_str());
            close(client_socket);

            break;
        }

        // Check if all data has been received
        data_received += bytes_received;
        if (data_received == data_size) {
            // Reset the number of bytes received
            data_received = 0;

            // Skip frame if no camera info was yet received.
            if (!depth_info.has_value()) {
                std::unique_lock ros_lock(ros_mutex);
                rclcpp::spin_some(ros_node);
                continue;
            }

            // Extract properties from camera info.
            uint32_t width = depth_info->width;
            uint32_t height = depth_info->height;
            cv::Matx44f proj(
                depth_info->p[0], depth_info->p[1], depth_info->p[2], depth_info->p[3],
                depth_info->p[4], depth_info->p[5], depth_info->p[6], depth_info->p[7],
                depth_info->p[8], depth_info->p[9], depth_info->p[10], depth_info->p[11],
                0, 0, 1, 0
            );
            cv::Matx44f proj_inv = proj.inv();

            // Split data into color and depth.
            color.data.resize(data.size() / 4 * 3);
            depth.data.resize(data.size() / 4);
            for (uint32_t row = 0; row < height; row++) {
                uint32_t row_flip = height - row - 1;
                for (uint32_t col = 0; col < width; col++) {
                    uint32_t i = row * width + col;
                    uint32_t i_flip = row_flip * width + col; // index of the corresponding pixel in a vertically flipped image

                    color.data[i_flip * 3 + 0] = data[i * 4 + 0];
                    color.data[i_flip * 3 + 1] = data[i * 4 + 1];
                    color.data[i_flip * 3 + 2] = data[i * 4 + 2];

                    depth.data[i_flip] = data[i * 4 + 3];
                }
            }

            // Initialize the rest of color and depth messages.
            color.header.frame_id = client_id + "_color_optical_frame";
            depth.header.frame_id = client_id + "_depth_optical_frame";
            color.header.stamp = depth.header.stamp = rclcpp::Clock().now();
            color.width = depth.width = width;
            color.height = depth.height = height;
            color.step = width * 3;
            depth.step = width;
            color.encoding = "rgb8";
            depth.encoding = "mono8";

            // Generate point cloud.
            size_t point_cloud_width = static_cast<float>(width) / height * point_cloud_height;
            point_cloud.data.reserve(point_cloud_width * point_cloud_height * 4 * 4); // 4 bytes per float; 4 floats per point (x, y, z, rgb); rgb is actually uint32
            point_cloud.data.clear();
            for (size_t row = 0; row < point_cloud_height; row++) {
                for (size_t col = 0; col < point_cloud_width; col++) {
                    size_t src_row = row * (static_cast<float>(height) / point_cloud_height) + 0.5;
                    size_t src_col = col * (static_cast<float>(height) / point_cloud_height) + 0.5;
                    
                    if (src_row >= height || src_col >= width) {
                        continue;
                    }

                    uint8_t src_depth = depth.data[src_row * width + src_col];
                    uint32_t src_r = color.data[(src_row * width + src_col) * 3 + 0];
                    uint32_t src_g = color.data[(src_row * width + src_col) * 3 + 1];
                    uint32_t src_b = color.data[(src_row * width + src_col) * 3 + 2];

                    if (src_depth != 0 && src_depth != 255) {
                        size_t i = point_cloud.data.size();
                        point_cloud.data.resize(point_cloud.data.size() + 16);

                        // Use inverse projection matrix to calculate point coordinates.
                        cv::Vec4f point = proj_inv * cv::Vec4f(src_col, src_row, src_depth / 255.0F, 1);
                        point /= point[3];
                        uint32_t rgb = (src_r & 0xff) << 16 | (src_g & 0xff) << 8 | (src_b & 0xff);
                        
                        // Add a point to the buffer.
                        *reinterpret_cast<float *>(&point_cloud.data[i + 0]) = point[0];
                        *reinterpret_cast<float *>(&point_cloud.data[i + 4]) = point[1];
                        *reinterpret_cast<float *>(&point_cloud.data[i + 8]) = point[2];
                        *reinterpret_cast<uint32_t *>(&point_cloud.data[i + 12]) = rgb;
                    }
                }
            }

            // Initialize the rest of the point cloud message.
            point_cloud.header.frame_id = client_id + "_depth_optical_frame";
            point_cloud.header.stamp = rclcpp::Clock().now();
            point_cloud.height = 1;
            point_cloud.width = point_cloud.data.size() / 16;
            point_cloud.fields.resize(4);
            point_cloud.fields[0].name = "x";
            point_cloud.fields[0].offset = 0;
            point_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            point_cloud.fields[0].count = 1;
            point_cloud.fields[1].name = "y";
            point_cloud.fields[1].offset = 4;
            point_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            point_cloud.fields[1].count = 1;
            point_cloud.fields[2].name = "z";
            point_cloud.fields[2].offset = 8;
            point_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            point_cloud.fields[2].count = 1;
            point_cloud.fields[3].name = "rgb";
            point_cloud.fields[3].offset = 12;
            point_cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
            point_cloud.fields[3].count = 1;
            point_cloud.is_bigendian = false;
            point_cloud.point_step = 16;
            point_cloud.row_step = point_cloud.data.size();
            point_cloud.is_dense = true;

            // Publish.
            {
                std::unique_lock ros_lock(ros_mutex);
                color_pub.publish(color);
                depth_pub.publish(depth);
                points_pub->publish(point_cloud);
                rclcpp::spin_some(ros_node);
            }

            // Measure frame time.
            float frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count() / 1000.0F;
            now = std::chrono::steady_clock::now();
            frame_time_smooth = frame_time_smooth * frame_time_smoothing + frame_time * (1 - frame_time_smoothing);

            // Log frame time.
            frame_time_log_countdown -= frame_time;
            if (frame_time_log_countdown <= 0) {
                RCLCPP_INFO(ros_node->get_logger(), "Client %s frame time: %d ms", client_id.c_str(), static_cast<int>(frame_time_smooth * 1000));
                frame_time_log_countdown = frame_time_log_interval;
            }
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
    ros_it = std::make_shared<image_transport::ImageTransport>(ros_node);

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
    strncpy(server_addr.sun_path, "/tmp/unity_rs_publisher", sizeof(server_addr.sun_path) - 1);

    // Bind the socket to the server address
    if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        RCLCPP_ERROR(ros_node->get_logger(), "Socket binding failed");
        close(server_socket);
        rclcpp::shutdown();
        return 1;
    }

    // Register a signal handler for Ctrl+C (SIGINT) + SIGTERM + SIGQUIT + SIGABRT + SIGHUP + SIGKILL
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

    RCLCPP_INFO(ros_node->get_logger(), "Server is listening for connections...");

    std::vector<std::thread> client_threads;

    while (true) {
        struct sockaddr_un client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        // Accept a new connection
        int client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_addr_len);
        if (client_socket == -1) {
            std::unique_lock ros_lock(ros_mutex);
            RCLCPP_ERROR(ros_node->get_logger(), "Socket accept failed");
            continue;
        }

        // Receive the buffer size and client ID as the handshake message
        char data_size_plus_client_id_buffer[256];
        int bytes_received = recv(client_socket, data_size_plus_client_id_buffer, sizeof(data_size_plus_client_id_buffer), 0);
        if (bytes_received <= 0) {
            close(client_socket);
            continue;
        }
        // Decode first 4 bytes as the buffer size
        int data_size = *(int*)data_size_plus_client_id_buffer;
        std::string client_id(data_size_plus_client_id_buffer + 4);


        // Create a new thread to handle the client
        client_threads.emplace_back(handle_client, client_socket, client_id, data_size);
    }

    return 0;
}
