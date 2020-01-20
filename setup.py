import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="lightfantastic", # Replace with your own username
    version="0.0.1",
    author="anfractuosity",
    description="Tag LEDs using a video",
    packages=setuptools.find_packages(),
    python_requires='>=3.5',
    install_requires=['numpy','opencv-python'],
    package_dir={'lightfantastic': 'lightfantastic'},
    scripts=['lightfantastic/lightfantastic']
)
