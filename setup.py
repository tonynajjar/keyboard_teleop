import os

from setuptools import setup
from glob import glob

package_name = "keyboard_teleop"
share_path = os.path.join("share", package_name)


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Tony Najjar",
    author_email="tony.najjar.1997@gmail.com",
    maintainer="Tony Najjar",
    maintainer_email="tony.najjar.1997@gmail.com",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop_incremental = keyboard_teleop.keyboard_teleop_incremental:main",
            "keyboard_teleop_hold = keyboard_teleop.keyboard_teleop_hold:main",
        ],
    },
)
