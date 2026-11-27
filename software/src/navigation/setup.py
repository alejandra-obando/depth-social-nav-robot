from setuptools import find_packages, setup

setup(
    name="navigation",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/navigation"]),
        ("share/navigation", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="State machine, follow and go-to-goal controllers, VFH obstacle avoidance and differential-drive kinematics. Plain Python, no ROS imports.",
    license="MIT",
    tests_require=["pytest"],
)
