import os
import shutil
import rclpy
import rclpy.node
import subprocess
import ament_index_python
import atexit


def find_unity_version(project_dir: str):
    # Read the version of the Unity project from ""./ProjectSettings/ProjectVersion.txt".
    with open(
        os.path.join(project_dir, "unity_sim/ProjectSettings/ProjectVersion.txt")
    ) as f:
        content = f.read()
        i = "".join(content).find("m_EditorVersion: ")
        if i == -1:
            raise RuntimeError("Failed to parse Unity project version.")
        version = content[i + len("m_EditorVersion: ") :].split("\n")[0]
        if version == "":
            raise RuntimeError("Failed to parse Unity project version.")

    return version


class UnitySim(rclpy.node.Node):
    def __init__(self):
        super().__init__("unity_sim")
        self.declare_parameter("scene", "Marsyard")

        # Find paths.
        with open(
            ament_index_python.packages.get_package_share_directory("unity_sim")
            + "/unity_project_dir"
        ) as f:
            project_dir = f.read()
        # (Either Distrobox host home or just $HOME without Distrobox.)
        running_in_distrobox = "DISTROBOX_HOST_HOME" in os.environ
        home = (
            os.environ.get("DISTROBOX_HOST_HOME")
            if running_in_distrobox
            else os.environ.get("HOME")
        )

        # Check if Native Plugin is installed and build it if it isn't.
        plugin_asset_path = f"{project_dir}/unity_sim/Assets/Simulation/RealSense/UnityRSPublisherPlugin.so"
        native_plugin_dir = os.path.abspath(project_dir + "/unity_rs_publisher_plugin")
        if not os.path.exists(plugin_asset_path):
            self.get_logger().info("Native plugin not found. Building it now...")

            # Build unity_rs_publisher_plugin on the host using CMake
            cmd_prefix = ["distrobox-host-exec"] if running_in_distrobox else []

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
                ]
            )
            if proc.returncode != 0:
                raise RuntimeError(
                    "Failed to configure CMake for unity_rs_publisher_plugin."
                )

            proc = subprocess.run(
                cmd_prefix
                + ["cmake", "--build", native_plugin_build_dir, "--config Release"]
            )
            if proc.returncode != 0:
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
        kill_cmd = ["pkill", "-9", "-f", " ".join(run_cmd)]

        # Run Unity in background.
        if running_in_distrobox:
            run_cmd = ["distrobox-host-exec"] + run_cmd
        self.get_logger().info(f"Running Unity with command: {' '.join(run_cmd)}")
        subprocess.Popen(run_cmd)

        # Spin until killed.
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down Unity:\n" + " ".join(kill_cmd))
            if running_in_distrobox:
                kill_cmd = ["distrobox-host-exec"] + kill_cmd
            subprocess.run(kill_cmd)
            exit(0)


# Simulation supervisor node.
def simulation(args=None):
    rclpy.init(args=args)

    UnitySim()

    rclpy.shutdown()


# Print the version of Unity.
def version(args=None):
    with open(
        ament_index_python.packages.get_package_share_directory("unity_sim")
        + "/unity_project_dir"
    ) as f:
        project_dir = f.read()
    version = find_unity_version(project_dir)
    print(f"Unity {version}")
