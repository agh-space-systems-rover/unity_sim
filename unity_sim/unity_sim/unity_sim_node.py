import os
import shutil
import rclpy
import rclpy.node
import subprocess
import signal
from ament_index_python import get_package_share_path
from unity_sim.util import find_unity_version


class UnitySim(rclpy.node.Node):
    def __init__(self):
        super().__init__("unity_sim")
        self.declare_parameter("scene", "ERC2023")

        # Find paths.
        project_dir = str(get_package_share_path("unity_sim") / "unity_project")
        # (Either Distrobox host home or just $HOME without Distrobox.)
        self.running_in_distrobox = "DISTROBOX_HOST_HOME" in os.environ
        home = (
            os.environ.get("DISTROBOX_HOST_HOME")
            if self.running_in_distrobox
            else os.environ.get("HOME")
        )

        # Check if Native Plugin is installed and build it if it isn't.
        plugin_asset_path = f"{project_dir}/unity_sim/Assets/Simulation/RealSense/UnityRSPublisherPlugin.so"
        native_plugin_dir = os.path.abspath(project_dir + "/unity_rs_publisher_plugin")
        if not os.path.exists(plugin_asset_path):
            self.get_logger().info("Native plugin not found. Building it now...")

            # Build unity_rs_publisher_plugin on the host using CMake
            cmd_prefix = ["distrobox-host-exec"] if self.running_in_distrobox else []

            native_plugin_build_dir = native_plugin_dir + "/build"
            os.makedirs(native_plugin_build_dir, exist_ok=True)

            proc = subprocess.run(
                cmd_prefix
                + [
                    "cmake",
                    "-B",
                    native_plugin_build_dir,
                    "-S",
                    native_plugin_dir,
                    "-DCMAKE_BUILD_TYPE=Release",
                ],
                capture_output=True,
                text=True,
            )
            if proc.returncode != 0:
                self.get_logger().error(
                    f"Failed to configure CMake for unity_rs_publisher_plugin.\n\n===STDOUT===\n{proc.stdout}\n===STDERR===\n{proc.stderr}\n===EXIT CODE===\n{proc.returncode}\n"
                )
                raise RuntimeError(
                    "Failed to configure CMake for unity_rs_publisher_plugin."
                )

            proc = subprocess.run(
                cmd_prefix
                + ["cmake", "--build", native_plugin_build_dir, "--config Release"],
                capture_output=True,
                text=True,
            )
            if proc.returncode != 0:
                self.get_logger().error(
                    f"Failed to build unity_rs_publisher_plugin.\n\n===STDOUT===\n{proc.stdout}\n===STDERR===\n{proc.stderr}\n===EXIT CODE===\n{proc.returncode}\n"
                )
                raise RuntimeError("Failed to build unity_rs_publisher_plugin.")

            shutil.copy(
                f"{native_plugin_build_dir}/libUnityRSPublisherPlugin.so",
                plugin_asset_path,
            )

        # Find Unity installation in ~/Unity/Hub/Editor/<version>.
        version = find_unity_version(project_dir)
        unity_dir = os.path.join(home, "Unity/Hub/Editor", version)
        if not os.path.isdir(unity_dir):
            raise RuntimeError(
                f"Failed to find Unity {version} at {unity_dir}.\nPlease install it on the host using Unity Hub.\nSee: https://unity.com/releases/editor/archive"
            )

        # Find the scene to run.
        scene = self.get_parameter("scene").get_parameter_value().string_value
        scene_path = os.path.normpath(
            f"{project_dir}/unity_sim/Assets/Simulation/Scenes/{scene}.unity"
        )

        # Prepare a command to run and kill Unity.
        run_cmd = [os.path.join(unity_dir, "Editor/Unity"), "-openfile", scene_path]
        # self.kill_cmd = ["pkill", "-9", "-f", '"' + " ".join(run_cmd) + '"']
        self.kill_cmd = ["pkill", "-9", "-f", "unity_sim/unity_project"]

        # Kill Unity before attempting to run it.
        print(" ".join(self.kill_cmd))

        # Run Unity in background.
        if self.running_in_distrobox:
            run_cmd = ["distrobox-host-exec"] + run_cmd
        self.get_logger().info(f"Running Unity with command: {' '.join(run_cmd)}")
        self.process = subprocess.Popen(
            run_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        self.terminating = False
        self.timer = self.create_timer(0.1, self.timer_callback)
        signal.signal(signal.SIGINT, self.sigint_handler)

    def sigint_handler(self, sig, frame):
        self.terminating = True

    def timer_callback(self):
        if self.terminating:
            self.kill_cmd += ["2>&1", ">", "/dev/null"]
            if self.running_in_distrobox:
                self.kill_cmd = ["distrobox-host-exec"] + self.kill_cmd
            self.get_logger().info("Shutting down Unity:\n" + " ".join(self.kill_cmd))
            os.system(" ".join(self.kill_cmd))
            raise KeyboardInterrupt
        elif self.process.poll() is not None:
            self.get_logger().info("Unity has exited.")
            raise KeyboardInterrupt


def main():
    try:
        rclpy.init()
        node = UnitySim()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
