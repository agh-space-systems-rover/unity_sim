import glob
import os
import sys
from setuptools import find_packages, setup
import xml.etree.ElementTree as ET

# Parse package.xml to extract package information.
tree = ET.parse("package.xml")
root = tree.getroot()
package_name = root.find("name").text
package_version = root.find("version").text
maintainer_name = root.find("maintainer").text
maintainer_email = root.find("maintainer").get("email")
description = root.find("description").text
license = root.find("license").text

pkg_dir = os.path.abspath(sys.path[0])
unity_project_dir = os.path.abspath(os.path.join(pkg_dir, "..", "unity_project"))

# This is a 🕵️ hack 🕵️ that allows us to find the root of the source directory during runtime.
# Save current source path to a temporary data file.
with open(os.path.join(pkg_dir, "unity_project_dir"), "w") as f:
    # Exploit the fact that the current directory is the root of the source directory.
    f.write(unity_project_dir)

try:
    setup(
        name=package_name,
        version=package_version,
        packages=find_packages(),
        data_files=[
            ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
            (
                "share/" + package_name,
                ["package.xml", "unity_project_dir"],
            ),  # Install the hack.
            (
                os.path.join("share", package_name, "launch"),
                glob.glob(os.path.join("launch", "*launch.[pxy][yma]*")),
            ),
        ],
        install_requires=[],
        zip_safe=True,
        maintainer=maintainer_name,
        maintainer_email=maintainer_email,
        description=description,
        license=license,
        tests_require=["pytest"],
        entry_points={
            "console_scripts": [
                "simulation = unity_sim:simulation",
                "version = unity_sim:version",
            ],
        },
    )
except Exception as e:
    # Remove temporary data file.
    os.remove(os.path.join(pkg_dir, "unity_project_dir"))
    raise e
os.remove(os.path.join(pkg_dir, "unity_project_dir"))
