from setuptools import find_packages, setup

package_name = "crazyflie_webots_examples"

data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))

data_files.append(
    ("share/" + package_name + "/launch", ["launch/crazyflie_webots_example.launch.py"])
)


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rosrunner",
    maintainer_email="rosrunner@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "crazyflie = crazyflie_webots_examples.crazyflie_example:main"
        ],
    },
)
