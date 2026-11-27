from setuptools import find_packages, setup

setup(
    name="perception",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/perception"]),
        ("share/perception", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Detection of the person's AprilTags, person tracking (Kalman) and obstacle points from the LiDAR and the depth camera. Plain Python, no ROS imports.",
    license="MIT",
    tests_require=["pytest"],
)
