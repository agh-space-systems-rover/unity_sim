#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

// This Unity plugin sends raw frames over /tmp/unity_rs_publisher Unix socket.

// std::ofstream log_file("/tmp/unity_rs_publisher_plugin.log");
// std::mutex    log_mutex;
// void // async_log(const std::string &message) {
// 	std::unique_lock<std::mutex> lock(log_mutex);
// 	log_file << message << std::endl;
// }

class real_sense {
public:
	std::string             id;
	int                     width, height;
	float                   vfov, depth_min, depth_max;
	std::thread             thread;
	std::mutex              mutex;
	std::condition_variable cv;
	std::vector<uint8_t>    main_thread_data;
	bool                    running       = true;
	bool                    has_frame     = false;
	int                     client_socket = -1;
};

std::map<std::string, real_sense> cameras;

#define API_EXPORT __attribute__((visibility("default")))
extern "C" {

API_EXPORT void CloseBridgeConnection(const char *id);

API_EXPORT void OpenBridgeConnection(
    const char *id,
    int         width,
    int         height,
    float       vfov,
    float       depth_min,
    float       depth_max
) {
	// async_log("OpenBridgeConnection");

	// Check if the camera is already connected.
	if (cameras.find(std::string{id}) != cameras.end()) {
		// async_log("Camera already connected");

		// Shut down the existing connection.
		CloseBridgeConnection(id);
		// Continue with the new connection.
	}

	std::string id_str{id};
	auto       &rs = cameras[id_str];
	rs.id          = id_str;
	rs.width       = width;
	rs.height      = height;
	rs.vfov        = vfov;
	rs.depth_min   = depth_min;
	rs.depth_max   = depth_max;

	rs.thread = std::thread([&rs]() {
		// async_log("Thread started");
		while (rs.running) {
			// async_log("Loop start");
			try {
				// async_log("Creating socket");
				// Create a socket
				rs.client_socket = socket(AF_UNIX, SOCK_STREAM, 0);
				if (rs.client_socket == -1) {
					// async_log("Socket creation failed, retrying in 1
					// second"); Retry after 1 second.
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;
				}

				struct sockaddr_un server_addr;
				memset(&server_addr, 0, sizeof(server_addr));
				server_addr.sun_family = AF_UNIX;
				strncpy(
				    server_addr.sun_path,
				    "/tmp/unity_rs_publisher",
				    sizeof(server_addr.sun_path) - 1
				);

				// async_log("Connecting to server");
				// Connect to the server
				if (connect(
				        rs.client_socket,
				        (struct sockaddr *)&server_addr,
				        sizeof(server_addr)
				    ) == -1) {
					// async_log("Connection failed, retrying in 1 second");
					// Retry after 1 second.
					close(rs.client_socket);
					rs.client_socket = -1;
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;
				}
				// async_log("Connected to server");

				// Send initial handshake message (first 256 bytes)
				std::vector<uint8_t> buffer(256);
				// Send frame buffer size in the first 4 bytes of the handshake
				// message.
				*reinterpret_cast<uint32_t *>(buffer.data()) =
				    8 + rs.width * rs.height * 4;
				// Follow up with the camera ID.
				std::copy(rs.id.begin(), rs.id.end(), buffer.begin() + 4);
				// Null-terminate the camera ID.
				buffer[rs.id.size() + 4] = '\0';

				// async_log("Sending handshake message");
				if (send(rs.client_socket, buffer.data(), buffer.size(), 0) ==
				    -1) {
					// async_log("Handshake failed, retrying in 1 second");
					// Retry after 1 second.
					close(rs.client_socket);
					rs.client_socket = -1;
					std::this_thread::sleep_for(std::chrono::seconds(1));
					continue;
				}
				// async_log("Handshake successful");

				std::vector<uint8_t> worker_data;

				while (rs.running) {
					// async_log("Waiting for frame");
					{
						std::unique_lock<std::mutex> lock(rs.mutex);
						rs.cv.wait(lock, [&rs] {
							return rs.has_frame || !rs.running;
						});

						// Exit if requested.
						if (!rs.running) {
							// async_log("Exiting thread");
							return;
						}

						// Swap the buffers.
						worker_data.swap(rs.main_thread_data);
						rs.has_frame = false;
					}
					// async_log("Frame received, sending data");

					// Send the message without null-terminating character
					if (send(
					        rs.client_socket,
					        worker_data.data(),
					        worker_data.size(),
					        0
					    ) == -1) {
						// async_log("Sending data failed, reconnecting");
						// Connection was probably closed by the server.
						// Need to break out of sending loop and try to
						// reconnect.
						break;
					}
					// async_log("Data sent successfully");
				}

				// async_log("Closing socket");
				close(rs.client_socket);
				rs.client_socket = -1;
			} catch (...) {
				// async_log("Exception caught, closing socket");
				if (rs.client_socket != -1) {
					close(rs.client_socket);
					rs.client_socket = -1;
				}
			}
			// async_log("Retrying in 1 second");
			// Retry after 1 second before attempting to reconnect.
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
		// async_log("Thread exiting");
	});
}

API_EXPORT void CloseBridgeConnection(const char *id) {
	// async_log("CloseBridgeConnection");
	auto it = cameras.find(std::string{id});
	if (it != cameras.end()) {
		auto &rs = it->second;
		// async_log("Closing connection");
		{
			// Wait for the job to finish.
			std::unique_lock<std::mutex> lock(rs.mutex);
			// Exit the worker thread.
			rs.running = false;
			rs.cv.notify_one();
		}
		// Join with the worker thread.
		rs.thread.join();
		if (rs.client_socket != -1) {
			// async_log("Closing socket");
			close(rs.client_socket);
			rs.client_socket = -1;
		}
		cameras.erase(it);
		// async_log("Connection closed");
	}
}

API_EXPORT void
TryPushFrame(const char *id, int32_t sec, uint32_t nanosec, void *ptr) {
	// async_log("TryPushFrame");
	auto it = cameras.find(std::string{id});
	if (it != cameras.end()) {
		// async_log("Pushing frame");
		auto    &rs    = it->second;
		uint8_t *bytes = reinterpret_cast<uint8_t *>(ptr);

		// Copy data to the buffer.
		{
			std::unique_lock<std::mutex> lock(rs.mutex);
			rs.main_thread_data.resize(2 * 4 + rs.width * rs.height * 4);

			// stamp
			*reinterpret_cast<int32_t *>(rs.main_thread_data.data()) = sec;
			*reinterpret_cast<uint32_t *>(rs.main_thread_data.data() + 4) =
			    nanosec;

			// frame data
			std::copy(
			    bytes,
			    bytes + rs.width * rs.height * 4,
			    rs.main_thread_data.begin() + 8
			);

			rs.has_frame = true;
		}

		// Notify the worker thread.
		rs.cv.notify_one();

		// async_log("Frame pushed");
		// Exit immediately without waiting for the worker thread to finish.
	}
}
}