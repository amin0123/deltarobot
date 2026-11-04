from glob import glob
from setuptools import setup

setup(name="delta_sorting", version="0.1.0", packages=["delta_sorting"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/delta_sorting"]),
                  ("share/delta_sorting", ["package.xml"]),
                  ("share/delta_sorting/launch", glob("launch/*.py")),
                  ("share/delta_sorting/config", glob("config/*"))],
      install_requires=["setuptools"], zip_safe=True,
      entry_points={"console_scripts": [
          "perception = delta_sorting.perception_node:main",
          "projection = delta_sorting.projection_node:main",
          "motion = delta_sorting.motion_node:main",
          "coordinator = delta_sorting.coordinator_node:main",
          "evaluate = delta_sorting.evaluation_node:main"]})
