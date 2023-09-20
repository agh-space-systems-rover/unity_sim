#include <cstdint>
#include <memory>
#include <map>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <cmath>

#include <hv/WebSocketClient.h>
#include <turbojpeg.h>
#include "json.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Function to encode binary data to base64
std::string base64_encode(uint8_t *data, size_t size) {
    const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    size_t input_size = size;
    size_t i = 0;

    // Calculate the size of the output string and reserve space
    size_t output_size = 4 * ((input_size + 2) / 3); // Includes padding
    std::string base64_str;
    base64_str.reserve(output_size);

    while (i < input_size) {
        uint8_t octet1 = i < input_size ? data[i++] : 0;
        uint8_t octet2 = i < input_size ? data[i++] : 0;
        uint8_t octet3 = i < input_size ? data[i++] : 0;

        uint32_t triplet = (static_cast<uint32_t>(octet1) << 16) |
                           (static_cast<uint32_t>(octet2) << 8) |
                           static_cast<uint32_t>(octet3);

        base64_str.push_back(base64_chars[(triplet >> 18) & 0x3F]);
        base64_str.push_back(base64_chars[(triplet >> 12) & 0x3F]);
        base64_str.push_back(base64_chars[(triplet >> 6) & 0x3F]);
        base64_str.push_back(base64_chars[triplet & 0x3F]);
    }

    // Handle padding if necessary
    size_t padding = 3 - (input_size % 3);
    while (padding > 0) {
        base64_str.push_back('=');
        padding--;
    }

    return base64_str;
}

class real_sense {
public:
	int width, height;
	float vfov, depth_min, depth_max;
    std::shared_ptr<hv::WebSocketClient> ws;
	std::thread thread;
	std::mutex mutex;
	std::condition_variable cv;
	std::vector<uint8_t> main_thread_data;
	bool running = true;
	bool has_frame = false;
};

std::map<std::string, real_sense> cameras;

#define API_EXPORT __attribute__((visibility("default")))
extern "C" {

API_EXPORT void OpenBridgeConnection(const char *id, int width, int height, int vfov, int depth_min, int depth_max) {
	std::string id_str{id};
	auto &rs = cameras[id_str];
	rs.width = width;
	rs.height = height;
	rs.vfov = vfov;
	rs.depth_min = depth_min;
	rs.depth_max = depth_max;
    rs.ws = std::make_shared<hv::WebSocketClient>();

    // Connect to realsense_bridge
    try {
        rs.ws->setPingInterval(10000);

        reconn_setting_t reconn;
        reconn_setting_init(&reconn);
        reconn.min_delay = 1000;
        reconn.max_delay = 10000;
        reconn.delay_policy = 2;
        rs.ws->setReconnect(&reconn);

		rs.ws->onopen = [&rs, id_str]() {
			rs.ws->send(nlohmann::json{
				{"op", "advertise"},
				{"type", "sensor_msgs/msg/CompressedImage"},
				{"topic", "/" + id_str + "/color/image_raw/compressed"},
			}.dump());
			rs.ws->send(nlohmann::json{
				{"op", "advertise"},
				{"type", "sensor_msgs/msg/CompressedImage"},
				{"topic", "/" + id_str + "/depth/image_rect_raw/compressed"},
			}.dump());
			rs.ws->send(nlohmann::json{
				{"op", "advertise"},
				{"type", "sensor_msgs/msg/PointCloud2"},
				{"topic", "/" + id_str + "/depth/color/points"},
			}.dump());
			rs.ws->send(nlohmann::json{
				{"op", "advertise"},
				{"type", "std_msgs/msg/String"},
				{"topic", "/" + id_str + "/unity_plugin_stats"},
			}.dump());
		};

        rs.ws->open("ws://localhost:9090");

		rs.thread = std::thread([&rs, id_str]() {
			std::vector<uint8_t> worker_data, color, depth; // color_jpeg, depth_jpeg;
			std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
			unsigned long color_jpeg_size = tjBufSize(rs.width, rs.height, TJSAMP_420);
			uint8_t *color_jpeg_buf = new uint8_t[color_jpeg_size];
			unsigned long depth_jpeg_size = tjBufSize(rs.width, rs.height, TJSAMP_GRAY);
			uint8_t *depth_jpeg_buf = new uint8_t[depth_jpeg_size];
			std::vector<uint8_t> point_cloud;

			// Try to initialize turbo.
			tjhandle tj = tjInitCompress();
			// Write status to file.
			std::ofstream ofs("/tmp/unity-native-plugin-turbo-init-status.log");
			if (tj) {
				ofs << "turbo initialized successfully" << std::endl;
			} else {
				ofs << "turbo failed to initialize: " << tjGetErrorStr() << std::endl;
			}
			ofs.close();

			while (true) {
				{
					std::unique_lock<std::mutex> lock(rs.mutex);
					rs.cv.wait(lock, [&rs] { return rs.has_frame || !rs.running; });

					// Exit if requested.
					if (!rs.running) {
						delete color_jpeg_buf;
						delete depth_jpeg_buf;
						break;
					}

					// Swap the buffers.
					worker_data.swap(rs.main_thread_data);
					rs.has_frame = false;
				}

				// Reset the timer.
				now = std::chrono::steady_clock::now();

// 				header: 
//   seq: 725
//   stamp: 
//     secs: 87
//     nsecs: 920000000
//   frame_id: "d455_left_front_depth_optical_frame"
// height: 480
// width: 640
// fields: 
//   - 
//     name: "x"
//     offset: 0
//     datatype: 7
//     count: 1
//   - 
//     name: "y"
//     offset: 4
//     datatype: 7
//     count: 1
//   - 
//     name: "z"
//     offset: 8
//     datatype: 7
//     count: 1
//   - 
//     name: "rgb"
//     offset: 16
//     datatype: 7
//     count: 1
// is_bigendian: False
// point_step: 32
// row_step: 20480
				
				// Split data into color and depth
				color.resize(worker_data.size() / 4 * 3);
				depth.resize(worker_data.size() / 4);
				for (int row = 0; row < rs.height; row++) {
					int row_flip = rs.height - row - 1;
					for (int col = 0; col < rs.width; col++) {
						int i = row * rs.width + col;
						int i_flip = row_flip * rs.width + col;
						color[i_flip * 3 + 0] = worker_data[i * 4 + 0];
						color[i_flip * 3 + 1] = worker_data[i * 4 + 1];
						color[i_flip * 3 + 2] = worker_data[i * 4 + 2];

						// if (row >= 40 && row < 760) {
						// 	depth[(row_flip - 40) * 1280 + col] = worker_data[i * 4 + 3];
						// }
						depth[i_flip] = worker_data[i * 4 + 3];
					}
				}

				// Remember splitting duration
				float split_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count();
				now = std::chrono::steady_clock::now();

				// // Compress color to JPEG
				// color_jpeg.reserve(65536); // May reallocate a few times on the first frame.
				// color_jpeg.clear();
				// stbi_write_jpg_to_func([](void *context, void *data, int size) {
				// 	auto &color_jpeg = *reinterpret_cast<std::vector<uint8_t> *>(context);
				// 	color_jpeg.insert(color_jpeg.end(), reinterpret_cast<uint8_t *>(data), reinterpret_cast<uint8_t *>(data) + size);
				// }, &color_jpeg, 1280, 800, 3, color.data(), 80);

				// // Compress depth to JPEG
				// depth_jpeg.reserve(65536); // May reallocate a few times on the first frame.
				// depth_jpeg.clear();
				// stbi_write_jpg_to_func([](void *context, void *data, int size) {
				// 	auto &depth_jpeg = *reinterpret_cast<std::vector<uint8_t> *>(context);
				// 	depth_jpeg.insert(depth_jpeg.end(), reinterpret_cast<uint8_t *>(data), reinterpret_cast<uint8_t *>(data) + size);
				// }, &depth_jpeg, 1280, 720, 1, depth.data(), 80);

				// Compress color to JPEG
				int error = tjCompress2(tj, color.data(), rs.width, 0, rs.height, TJPF_RGB, &color_jpeg_buf, &color_jpeg_size, TJSAMP_420, 80, TJFLAG_FASTDCT | TJFLAG_NOREALLOC);
				if (error) {
					std::ofstream ofs("/tmp/unity-native-plugin-turbo-color-compress-error.log");
					ofs << tjGetErrorStr() << std::endl;
					ofs.close();
				}

				// Compress depth to JPEG; Note: Due to a possible bug, Turbo is unable to compress grayscale with reallocation turned on.
				error = tjCompress2(tj, depth.data(), rs.width, 0, rs.height, TJPF_GRAY, &depth_jpeg_buf, &depth_jpeg_size, TJSAMP_GRAY, 80, TJFLAG_FASTDCT | TJFLAG_NOREALLOC);
				if (error) {
					std::ofstream ofs("/tmp/unity-native-plugin-turbo-depth-compress-error.log");
					ofs << tjGetErrorStr() << std::endl;
					ofs.close();
				}

				// Remember compression duration
				float compress_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count();
				now = std::chrono::steady_clock::now();
				
				// Generate point cloud.
				size_t point_cloud_width = rs.width / 8;
				size_t point_cloud_height = rs.height / 8;
				point_cloud.reserve(point_cloud_width * point_cloud_height * 4 * 4); // 4 bytes per float; 4 floats per point (x, y, z, rgb); rgb is actually uint32
				point_cloud.clear();
				for (int row = 0; row < point_cloud_height; row++) {
					for (int col = 0; col < point_cloud_width; col++) {
						int src_row = row * (rs.height / point_cloud_height);
						int src_col = col * (rs.width / point_cloud_width);
						uint8_t src_depth = depth[src_row * rs.width + src_col];
						uint32_t src_r = color[src_row * rs.width + src_col * 3 + 0];
						uint32_t src_g = color[src_row * rs.width + src_col * 3 + 1];
						uint32_t src_b = color[src_row * rs.width + src_col * 3 + 2];

						if (src_depth != 0) {
							size_t i = point_cloud.size();
							point_cloud.resize(point_cloud.size() + 16);

							float z = static_cast<float>(src_depth) / 255.0 * (rs.depth_max - rs.depth_min) + rs.depth_min;
							float u = static_cast<float>(col) / rs.width * 2 - 1;
							float v = static_cast<float>(row) / rs.height  * 2 - 1;
							float hfov = rs.width / rs.height * rs.vfov;
							float x = std::tan((hfov / 2) * 3.14159F / 180) * z;
							float y = std::tan((rs.vfov / 2) * 3.14159F / 180) * z;
							// float r = static_cast<float>(src_r) / 255.0;
							// float g = static_cast<float>(src_g) / 255.0;
							// float b = static_cast<float>(src_b) / 255.0;
							uint32_t rgb = (src_r << 16) | (src_g << 8) | src_b;
							
							// Add a point to the buffer.
							*reinterpret_cast<float *>(&point_cloud[i + 0]) = x;
							*reinterpret_cast<float *>(&point_cloud[i + 4]) = y;
							*reinterpret_cast<float *>(&point_cloud[i + 8]) = z;
							*reinterpret_cast<uint32_t *>(&point_cloud[i + 12]) = rgb;
						}
					}
				}

				// Remember point cloud generation duration
				float point_cloud_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count();
				now = std::chrono::steady_clock::now();

				// Find current unix timestamp and assembly the header.
				auto epoch = std::chrono::system_clock().now();
				auto epoch_sec = std::chrono::time_point_cast<std::chrono::seconds>(epoch);
				auto epoch_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - epoch_sec);
				nlohmann::json stamp{
					{"sec", epoch_sec.time_since_epoch().count()},
					{"nanosec", epoch_nanosec.count()}
				};

				// Send data to rosbridge
				if (rs.ws->isConnected()) {
					// Send color.
					rs.ws->send(nlohmann::json{
						{"op", "publish"},
						{"topic", "/" + id_str + "/color/image_raw/compressed"},
						{"msg", {
							{"header", {
								{"stamp", stamp},
								{"frame_id", id_str + "_color_frame"},
							}},
							{"format", "jpeg"},
							{"data", base64_encode(color_jpeg_buf, color_jpeg_size)}
						}},
					}.dump());
					
					// Send depth.
					rs.ws->send(nlohmann::json{
						{"op", "publish"},
						{"topic", "/" + id_str + "/depth/image_rect_raw/compressed"},
						{"msg", {
							{"header", {
								{"stamp", stamp},
								{"frame_id", id_str + "_depth_frame"},
							}},
							{"format", "jpeg"},
							{"data", base64_encode(depth_jpeg_buf, depth_jpeg_size)}
						}},
					}.dump());

					// Send point cloud.
					rs.ws->send(nlohmann::json{
						{"op", "publish"},
						{"topic", "/" + id_str + "/depth/color/points"},
						{"msg", {
							{"header", {
								{"stamp", stamp},
								{"frame_id", id_str + "_depth_frame"},
							}},
							{"height", 1},
							{"width", point_cloud.size() / 16},
							{"fields", {
								{
									{"name", "x"},
									{"offset", 0},
									{"datatype", 7},
									{"count", 1},
								},
								{
									{"name", "y"},
									{"offset", 4},
									{"datatype", 7},
									{"count", 1},
								},
								{
									{"name", "z"},
									{"offset", 8},
									{"datatype", 7},
									{"count", 1},
								},
								{
									{"name", "rgb"},
									{"offset", 12},
									{"datatype", 6},
									{"count", 1},
								},
							}},
							{"is_bigendian", false},
							{"point_step", 16},
							{"row_step", point_cloud.size()},
							{"data", base64_encode(point_cloud.data(), point_cloud.size())},
							{"is_dense", true},
						}},
					}.dump());

					// Remember sending duration.
					float send_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count();
					now = std::chrono::steady_clock::now();

					// Send profiling message.
					rs.ws->send(nlohmann::json{
						{"op", "publish"},
						{"topic", "/" + id_str + "/unity_plugin_stats"},
						{"msg", {
							{"data", "split = " + std::to_string(split_time) + " ms, compress = " + std::to_string(compress_time) + " ms, point cloud = " + std::to_string(point_cloud_time) + " ms, send = " + std::to_string(send_time) + " ms, total = " + std::to_string(split_time + compress_time + send_time) + " ms"},
						}},
					}.dump());
				}
			}
		});
    } catch (...) {}
}

API_EXPORT void CloseBridgeConnection(const char *id) {
    try {
        auto &rs = cameras.at(std::string{id});
		{
			// Wait for the job to finish.
			std::unique_lock<std::mutex> lock(rs.mutex);
			// Exit the worker thread.
			rs.running = false;
			rs.cv.notify_one();
		}
		// Join with the worker thread.
		rs.thread.join();
        try {
            // rs->ws->stop();
            rs.ws->close();
        } catch (...) {}
        cameras.erase(std::string{id});
    } catch (...) {}
}

API_EXPORT void TryPushFrame(const char *id, void *ptr) {
    try {
        auto &rs = cameras.at(std::string{id});
		uint8_t *bytes = reinterpret_cast<uint8_t *>(ptr);

		// Copy data to the buffer.
		{
			std::unique_lock<std::mutex> lock(rs.mutex);
			rs.main_thread_data.resize(rs.width * rs.height * 4);
			std::copy(bytes, bytes + rs.main_thread_data.size(), rs.main_thread_data.begin());
			rs.has_frame = true;
		}
		
		// Notify the worker thread.
		rs.cv.notify_one();

		// Exit immediately without waiting for the worker thread to finish.
    } catch (...) {}
}

}
