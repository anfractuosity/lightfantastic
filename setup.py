import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="lightfantastic", 
    version="0.0.1",
    author="anfractuosity",
    description="Tag LEDs using a video",
    python_requires='>=3.5',
    install_requires=['numpy','opencv-python'],
    packages=setuptools.find_packages(),
    package_dir={'lightfantastic': 'lightfantastic'},
    scripts=['lightfantastic-calibrate', 'lightfantastic-process']
)
