from glob import glob

from setuptools import find_packages, setup

package_name = "fire_inspection_audio"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # 警报样本 wav 一并装进 install，运行时按包 share 目录读取
        ("share/" + package_name + "/config", glob("config/*.wav")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@nvidia.com",
    description="消防巡检听觉检测：本地音频指纹匹配防空警报。",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "audio_scan_node = fire_inspection_audio.audio_scan_node:main",
        ],
    },
)
