import os

from setuptools import find_packages, setup

package_name = "ugv_formation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/formation.launch.py"]),
        ("share/" + package_name + "/urdf", ["urdf/ugv02.urdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="UGV Formation Control",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["consensus_node = ugv_formation.consensus_node:main"],
    },
)
