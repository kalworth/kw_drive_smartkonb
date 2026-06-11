from setuptools import find_packages, setup

package_name = "kw_drive_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/kw_scope.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="Standalone plotting tools for KW Drive ROS 2 feedback.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "kw_scope = kw_drive_tools.kw_scope:main",
        ],
    },
)
