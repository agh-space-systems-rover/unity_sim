import glob
import os, sys
from setuptools import find_packages, setup

package_name = 'unity_sim'

# This is a 🕵️ hack 🕵️ that allows us to find the root of the source directory during runtime.
# Save current source path to a temporary data file.
with open(os.path.join(os.path.abspath(sys.path[0]), 'src-root'), 'w') as f:
    # Exploit the fact that the current directory is the root of the source directory.
    f.write(os.path.abspath(sys.path[0]))

try:
    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml', 'src-root']),
            (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*')))
        ],
        zip_safe=True,
        maintainer='rayferric',
        maintainer_email='63957587+rayferric@users.noreply.github.com',
        description='Unity simulation environment for AGH Space Systems robotics projects.',
        license='MIT',
        entry_points={
            'console_scripts': [
                'simulation = unity_sim:simulation',
                'version = unity_sim:version',
            ],
        },
    )
except Exception as e:
    # Remove temporary data file.
    os.remove(os.path.join(os.path.abspath(sys.path[0]), 'src-root'))
    raise e
os.remove(os.path.join(os.path.abspath(sys.path[0]), 'src-root'))
