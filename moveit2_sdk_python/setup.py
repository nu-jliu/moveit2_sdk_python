from setuptools import find_packages, setup

package_name = "moveit2_sdk_python"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    py_modules=[f"{package_name}.moveit2_sdk_python"],
    zip_safe=True,
    maintainer="jingkun",
    maintainer_email="jingkunliu2025@u.northwestern.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"try_node = {package_name}.try_node:main"],
    },
)
