import setuptools

with open("README.md", "r") as fh:
  long_description = fh.read()

setuptools.setup(
  name="bionic_dl",
  version="0.0.1",
  author="Siyu ZHANG",
  author_email="derizhangzsy@gmail.com",
  description="Package for data interfaces for robotics",
  long_description=long_description,
  long_description_content_type="text/markdown",
  url="https://github.com/ancorasir/flange_handeye_calibration",
  packages=setuptools.find_packages(),
  classifiers=[
  "Programming Language :: Python :: 3",
  "License :: OSI Approved :: MIT License",
  "Operating System :: Ubuntu",
  ],
)
