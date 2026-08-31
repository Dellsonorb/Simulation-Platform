from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    packages=['ground_pick_orchestrator'],
    package_dir={'': 'src'},
))
