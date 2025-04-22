from setuptools import setup

package_name = "mechanical_press"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="Mechanical press application node for ROS 2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mechanical_press = mechanical_press.mechanical_press:main",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
)

# [options.package_data]
# * = *.yaml

# [options.data_files]
# share/ament_index/resource_index/packages = resource/mechanical_press
# share/mechanical_press = package.xml
# share/mechanical_press/launch = launch/*.py
