import os
from glob import glob
from setuptools import find_packages, setup

package_name = "dt_risk_mpc_framework"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
        (os.path.join("share", package_name, "worlds"), glob(os.path.join("worlds", "*.*"))),
        (os.path.join("share", package_name, "models", "turtlebot4_black"), glob(os.path.join("models", "turtlebot4_black", "*.*"))),
        (os.path.join("share", package_name, "models", "turtlebot4_green"), glob(os.path.join("models", "turtlebot4_green", "*.*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="majdi-hassan",
    maintainer_email="majdiadam00@gmail.com",
    description="DT-Risk MPC framework for mobile robot navigation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dt_risk_aware_mpc = dt_risk_mpc_framework.DT_Risk_Aware_MPC:main",
            "dynamic_obstacles_mover = dt_risk_mpc_framework.dynamic_obstacles_mover:main",
        ],
    },
)
