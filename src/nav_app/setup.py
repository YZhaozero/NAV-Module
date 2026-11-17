import os
from glob import glob
from setuptools import find_packages, setup


package_name = "nav_app"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"nav_app": ["phone_web/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="zy",
    maintainer_email="2293086836@qq.com",
    description="Camera augmented-reality navigation application for ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ar_nav_node = nav_app.ar_nav_node:main",
            "qt_nav_node = nav_app.runtime:main",
            "reliable_record = nav_app.record_guard:main",
            "audit_bag = nav_app.features.bag_audit:main",
            "image_archive_play = nav_app.image_archive_play:main",
            "task_worker = nav_app.task_worker:main",
            "motion_guard = nav_app.motion_guard_node:main",
            "ros2_base_adapter = nav_app.ros2_base_adapter:main",
            "woosh_imu = nav_app.woosh_imu:main",
            "woosh_odometry = nav_app.woosh_odometry:main",
            "woosh_base_adapter = nav_app.woosh_base_adapter:main",
        ],
    },
)
