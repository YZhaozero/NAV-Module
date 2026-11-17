from glob import glob
from setuptools import find_packages, setup

setup(name="nav_rtk", version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/nav_rtk"]),
                  ("share/nav_rtk", ["package.xml", "LICENSE"]),
                  ("share/nav_rtk/launch", glob("launch/*.launch.py")),
                  ("share/nav_rtk/config", glob("config/*.yaml"))],
      install_requires=["setuptools", "numpy"], license="Apache-2.0",
      entry_points={"console_scripts": [
          "rtk_node = nav_rtk.bridge.rtk_node:main",
          "gnss_monitor = nav_rtk.gnss.node:main",
          "gnss_estimate_offset = nav_rtk.gnss.clock_cli:main"]})
