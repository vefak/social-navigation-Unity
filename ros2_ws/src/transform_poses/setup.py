from setuptools import find_packages, setup

package_name = "transform_poses"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="akmanadm",
    maintainer_email="vmakman@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf2_pose = transform_poses.tf2_pose:main",
            "groups = transform_poses.groups:main",
        ],
    },
)
