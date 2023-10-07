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
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// This Unity plugin sends raw frames over /tmp/unity_rs_publisher Unix socket.

class real_sense {
public:
	std::string id;
	int width, height;
	float vfov, depth_min, depth_max;
	std::thread thread;
	std::mutex mutex;
	std::condition_variable cv;
	std::vector<uint8_t> main_thread_data;
	bool running = true;
	bool has_frame = false;
	int client_socket;
};

std::map<std::string, real_sense> cameras;

#define API_EXPORT __attribute__((visibility("default")))
extern "C" {

API_EXPORT void OpenBridgeConnection(const char *id, int width, int height, float vfov, float depth_min, float depth_max) {
    std::string id_str{id};
	auto &rs = cameras[id_str];
	rs.id = id_str;
	rs.width = width;
	rs.height = height;
	rs.vfov = vfov;
	rs.depth_min = depth_min;
	rs.depth_max = depth_max;

	
	rs.thread = std::thread([&rs]() {
		try {
			// Create a socket
			rs.client_socket = socket(AF_UNIX, SOCK_STREAM, 0);
			if (rs.client_socket == -1) {
				perror("Socket creation failed");
				return;
			}

			struct sockaddr_un server_addr;
			memset(&server_addr, 0, sizeof(server_addr));
			server_addr.sun_family = AF_UNIX;
			strncpy(server_addr.sun_path, "/tmp/unity_rs_publisher", sizeof(server_addr.sun_path) - 1);

			// Connect to the server
			if (connect(rs.client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
				perror("Connection failed");
				close(rs.client_socket);
				return;
			}

			// Send initial handshake message (first 256 bytes)
			std::vector<uint8_t> buffer(256);
			// Send frame buffer size in the first 4 bytes of the handshake message.
			*reinterpret_cast<uint32_t *>(buffer.data()) = 8 + rs.width * rs.height * 4;
			// Follow up with the camera ID.
			std::copy(rs.id.begin(), rs.id.end(), buffer.begin() + 4);
			// Null-terminate the camera ID.
			buffer[rs.id.size() + 4] = '\0';

			if (send(rs.client_socket, buffer.data(), buffer.size(), 0) == -1) {
				perror("Handshake failed");
				close(rs.client_socket);
				return;
			}

			std::vector<uint8_t> worker_data;

			while (true) {
				{
					std::unique_lock<std::mutex> lock(rs.mutex);
					rs.cv.wait(lock, [&rs] { return rs.has_frame || !rs.running; });

					// Exit if requested.
					if (!rs.running) {
						close(rs.client_socket);
						break;
					}

					// Swap the buffers.
					worker_data.swap(rs.main_thread_data);
					rs.has_frame = false;
				}
				
				// Send the message without null-terminating character
				if (send(rs.client_socket, worker_data.data(), worker_data.size(), 0) == -1) {
					perror("Send failed");
					close(rs.client_socket);
					return;
				}
			}
		} catch (...) {
			close(rs.client_socket);
		}
	});
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
        cameras.erase(std::string{id});
    } catch (...) {}
}

API_EXPORT void TryPushFrame(const char *id, int32_t sec, uint32_t nanosec, void *ptr) {
    try {
        auto &rs = cameras.at(std::string{id});
		uint8_t *bytes = reinterpret_cast<uint8_t *>(ptr);

		// Copy data to the buffer.
		{
			std::unique_lock<std::mutex> lock(rs.mutex);
			rs.main_thread_data.resize(2 * 4 + rs.width * rs.height * 4);
			
			// stamp
			*reinterpret_cast<int32_t *>(rs.main_thread_data.data()) = sec;
			*reinterpret_cast<uint32_t *>(rs.main_thread_data.data() + 4) = nanosec;

			// frame data
			std::copy(bytes, bytes + rs.width * rs.height * 4, rs.main_thread_data.begin() + 8);

			rs.has_frame = true;
		}
		
		// Notify the worker thread.
		rs.cv.notify_one();

		// Exit immediately without waiting for the worker thread to finish.
    } catch (...) {}
}

}
