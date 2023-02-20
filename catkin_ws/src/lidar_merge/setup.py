from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = 'lidar_merge'

d = generate_distutils_setup(
    packages=[package_name],
    package_dir={package_name: package_name}
)

setup(**d)
