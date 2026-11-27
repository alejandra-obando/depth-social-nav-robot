from setuptools import find_packages, setup

setup(
    name="corners",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/corners"]),
        ("share/corners", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Wall segments (split-and-merge) and room-corner recognition from the LiDAR scan. Plain Python, no ROS imports.",
    license="MIT",
    tests_require=["pytest"],
)
