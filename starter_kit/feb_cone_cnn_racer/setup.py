import os
from glob import glob
from setuptools import setup

package_name = "feb_cone_cnn_racer"
setup(
    name=package_name, version="0.1.0", packages=[],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml", "card.yaml"]),
                (os.path.join("share", package_name, "launch"), glob("launch/*.py"))],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FEB autonomous team", maintainer_email="feb@berkeley.edu",
    description="feb_cone_racer with a trained cone detector", license="MIT",
    entry_points={"console_scripts": []},
)
