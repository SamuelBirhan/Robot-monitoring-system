from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_monitoring_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Package launch files
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        
        # If you have config files, uncomment this block
        # (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot monitoring package for ROS 2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Specify your entry points for your ROS 2 nodes here
            'error_handler = my_monitoring_package.error_handler:main',
            'state_monitor = my_monitoring_package.state_monitor:main',
            'sensor = my_monitoring_package.sensor:main',  # Add sensor entry point if necessary
        ],
    },
)
