from glob import glob
import os

from setuptools import find_packages, setup

package_name = "fire_inspection_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@example.com",
    description="消防巡检视觉服务端：InspectPoi Action",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "inspect_action_server = fire_inspection_vision.inspect_action_server:main",
            "vision_scan_node = fire_inspection_vision.vision_scan_node:main",
        ],
    },
)
