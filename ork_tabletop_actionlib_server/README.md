Steps to test this package:

term1: Start the robot
roslaunch /etc/ros/robot.launch

term2: Checkout shared_autonomy_perception
cd catkin_hydro
git clone git@github.com:SharedAutonomyToolkit/shared_autonomy_perception.git -b feature_ork_actionlib_server

term3: Install and run ORK
mkdir catkin_hydro && cd catkin_hydro
wstool init src https://raw.github.com/wg-perception/object_recognition_core/master/doc/source/ork.rosinstall
cd src && wstool update -j8
cd .. && rosdep install --from-paths src -i -y
catkin_make
source devel/setup.bash
roscd object_recognition_tabletop
patch -p1 -i `rospack find ork_tabletop_actionlib_server`/patch/clusters.patch
cd catkin_hydro
catkin_make
rosrun object_recognition_core detection -c `rospack find object_recognition_tabletop`/conf/detection.object.ros.ork

term4: Start Kinect driver
roslaunch shared_autonomy_launch openni_segmentation.launch

term5: Run ORK actionlib server
rosrun ork_tabletop_actionlib_server ork_tabletop_actionlib_server.py

term6: Run test client
rosrun ork_tabletop_actionlib_server test_client.py

