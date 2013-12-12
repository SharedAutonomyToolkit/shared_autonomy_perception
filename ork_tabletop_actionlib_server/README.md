Steps to test this package:

### Setup 

Install ORK:

  * mkdir catkin_hydro && cd catkin_hydro
  * wstool init src https://raw.github.com/wg-perception/object_recognition_core/master/doc/source/ork.rosinstall
  * cd src && wstool update -j8
  * cd .. && rosdep install --from-paths src -i -y

Install shared_autonomy_perception:

  * `git clone git@github.com:SharedAutonomyToolkit/shared_autonomy_perception.git`
  * `git clone git@github.com:bosch-ros-pkg/cluster_grasp_planner.git -b hydro-devel`

Patch ORK:

  * source ~catkin_hydro/devel/setup.bash
  * roscd object_recognition_tabletop
  * patch -p1 -i \`rospack find ork_tabletop_actionlib_server\`/patch/clusters.patch
  * cd ~catkin_hydro
  * catkin_make

### Execution:

  * robot_term1: `roslaunch /etc/ros/robot.launch`
  * robot_term2: rosrun object_recognition_core detection -c \`rospack find object_recognition_tabletop\`/conf/detection.object.ros.ork
  * robot_term3: `roslaunch shared_autonomy_launch openni_segmentation.launch`
  * robot_term4: `rosrun ork_tabletop_actionlib_server ork_tabletop_actionlib_server.py`
  * robot_term5: `rosrun ork_tabletop_actionlib_server test_client.py`

