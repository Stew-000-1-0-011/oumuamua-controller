from setuptools import find_packages, setup

package_name = "oumuamua_controller_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "msgpack"],
    zip_safe=True,
    maintainer="oumuamua",
    maintainer_email="noreply@anthropic.com",
    description="UDP/MessagePack bridge between the oumuamua Android controller and ROS 2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = oumuamua_controller_bridge.node:main",
            "fake_robot = oumuamua_controller_bridge.fake_robot:main",
        ],
    },
)
