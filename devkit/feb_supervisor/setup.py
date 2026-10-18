from setuptools import setup

package_name = "feb_supervisor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Formula Electric at Berkeley",
    maintainer_email="pranavbhatttheonlyone@gmail.com",
    description="The vehicle supervisor: safety state machine between every driver and the wheels",
    license="BSD-2-Clause",
    entry_points={"console_scripts": ["supervisor = feb_supervisor.supervisor:main"]},
)
