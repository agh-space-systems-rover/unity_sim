from ament_index_python import get_package_share_path
from unity_sim.util import find_unity_version


def main():
    project_dir = str(get_package_share_path("unity_sim") / "unity_project")
    version = find_unity_version(project_dir)
    print(f"Unity {version}")
