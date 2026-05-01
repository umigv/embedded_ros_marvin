from setuptools import find_packages, setup

package_name = "led_driver"

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
    maintainer="ericb",
    maintainer_email="ericbi@umich.edu",
    description="LED indicator driver node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "led_driver = led_driver.led_driver:main",
        ],
    },
)
