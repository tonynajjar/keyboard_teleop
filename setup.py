import os

from setuptools import setup
from glob import glob

package_name = "teleop_twist_keyboard"
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
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_twist_keyboard_incremental = teleop_twist_keyboard.teleop_twist_keyboard_incremental:main",
            "teleop_twist_keyboard_hold = teleop_twist_keyboard.teleop_twist_keyboard_hold:main",
        ],
    },
)
