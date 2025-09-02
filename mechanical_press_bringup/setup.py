from setuptools import setup
from glob import glob

package_name = "mechanical_press_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],  # no python modules required for bringup
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emoco",
    maintainer_email="ops@example.com",
    description="Launch and config for mechanical press",
    license="Apache-2.0",
)
