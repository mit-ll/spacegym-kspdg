from setuptools import setup, find_packages

def get_version():
    path = "src/kspdg/version.py"
    with open(path) as file:
        lines = file.readlines()

    for line in lines:
        if line.startswith("VERSION"):
            return line.strip().split("=")[-1].strip().strip('"')
    raise RuntimeError("bad version data in __init__.py")

setup(name="kspdg", 
      version=get_version(),
      packages=find_packages('src'),
      package_dir={'': 'src'},
      python_requires=">=3",
      install_requires=[
        "setuptools<=57.5.0", # pin setuptools to avoid error with use_2to3 in krpc
        "protobuf<=3.20", # pin to avoid major version change at 4.21 that breaks krpc
        "numpy",
        "gym",
        "poliastro",
      ]
      )
