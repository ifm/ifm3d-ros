from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# reads package.xml
setup_args = generate_distutils_setup(
    packages=['ifm3d_ros_utils'],
    package_dir={'': 'utils'},
    )

setup(**setup_args)
