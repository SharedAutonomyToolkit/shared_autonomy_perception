from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ork_tabletop_actionlib_server'],
    package_dir={'': 'src'},
    requires=['actionlib', 'actionlib_msgs', 'geometry_msgs', 'rospy', 'tf', 'object_recognition_msgs', 'sensor_msgs', 'visualization_msgs', 'shared_autonomy_msgs'],
    scripts=['scripts/actionlib_server.py']
)

setup(**d)
