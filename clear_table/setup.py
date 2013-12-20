from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['clear_table'],
    package_dir={'': 'src'},
    requires=['rospy', 'moveit_msgs', 'sensor_msgs', 'shape_msgs', 'shared_autonomy_msgs', 'smach', 'smach_ros', 'cluster_grasp_planner'],
    scripts=['scripts/clear_table_unified.py']
)

setup(**d)
