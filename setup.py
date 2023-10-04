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
        "setuptools",
        "krpc",
        "numpy",
        "gymnasium",
        "poliastro",
      ]
      )
