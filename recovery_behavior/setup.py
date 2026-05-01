from setuptools import find_packages, setup

package_name = "recovery_behavior"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hannah Hysell",
    maintainer_email="hannerd@umich.edu",
    description="Recovery behavior node with ultrasonic obstacle avoidance",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "recovery_behavior = recovery_behavior.recovery_executable:main",
        ],
    },
)
