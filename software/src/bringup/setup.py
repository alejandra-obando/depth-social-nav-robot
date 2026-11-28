from glob import glob

from setuptools import find_packages, setup

setup(
    name="bringup",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bringup"]),
        ("share/bringup", ["package.xml"]),
        ("share/bringup/launch", glob("launch/*.launch.py")),
        ("share/bringup/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="ROS 2 nodes (controller and speaker), launch file and parameters.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "social_nav_node = bringup.social_nav_node:main",
            "speaker_node = bringup.speaker_node:main",
            "make_voice = bringup.sounds:main",
        ],
    },
)
