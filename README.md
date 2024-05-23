# Supporting Files for Paper: RCM-constrained over-actuated manipulator tracking using differential kinematics control


Requirements
-----------

iiwa_ros requires several packages to be installed in order to work properly:

* [ROS] - tested in **Noetic**
* [iiwa_ros] - ROS Meta-package for controlling KUKA IIWA with a modified FRI Library


Compilation
------------

Add the files in this repositry to your ros_package then compile the package using

```sh
cd /path/to/ros_workspace
# source ros workspace
catkin_make
```

Controller Usage 
--------------

1. Make sure your Linux/ROS laptop is connected on the KONI Ethernet port and has IP `192.170.10.1` mask `255.255.255.0`.
2. On the Smartpad tablet:

* Activate `AUT` mode (turn key right > AUT > key left)
* In `[Application]`, check yours in order to select it
* Press the mechanical `Play` button ▶

3. Within 10 seconds before the timeout, launch: `roslaunch iiwa_driver iiwa_bringup.launch`. This will connect to IIWA robot using FRI.
4 The Smartpad lets you select control mode and stiffness
5. Check that everything works if `/iiwa/joint_states` is being published and reflects the actual robot state.
6. The Smartpad [Application] tab must remain green. Otherwise you can press `Play ▶` again to reconnect.
7. Once you connect to IIWA robot using FRI, run: `rosrun your_workspace RCM_Control.launch`.

Acknowledgements
---------------------
The modified [iiwa_stack] library used was created by Konstantinos Chatzilygeroudis (costashatz@gmail.com)

Authors/Maintainers
---------------------

- Omar Rayyan (olr7742@nyu.edu)
- Vinicius Goncalves (vmg6973@nyu.edu)
- Nikolaos Evangeliou (nikolaos.evangeliou@nyu.edu)


[ROS]: http://www.ros.org
[iiwa_ros]: https://github.com/epfl-lasa/iiwa_ros/tree/master
[gazebo]: http://gazebosim.org/
[ros control]: http://wiki.ros.org/ros_control
[kuka fri]: https://github.com/costashatz/kuka_fri
[spacevecalg]: https://github.com/jrl-umi3218/SpaceVecAlg
[rbdyn]: https://github.com/jrl-umi3218/RBDyn
[mc_rbdyn_urdf]: https://github.com/jrl-umi3218/mc_rbdyn_urdf
[robot_controllers]: https://github.com/epfl-lasa/robot_controllers
[corrade]: https://github.com/mosra/corrade
[iiwa_stack]: https://github.com/IFL-CAMP/iiwa_stack
