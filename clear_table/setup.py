from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['clear_table'],
    package_dir={'': 'src'},
    requires=['rospy', 'moveit_msgs', 'sensor_msgs', 'shape_msgs', 'shared_autonomy_msgs', 'smach', 'smach_ros'],
    scripts=['scripts/clear_table_moveit.py', 'scripts/clear_table3.py']
)

setup(**d)
