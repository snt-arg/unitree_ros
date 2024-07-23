^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package unitree_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#30 <https://github.com/snt-arg/unitree_ros/issues/30>`_ from snt-arg/feat/pub_joint_states
  Feature: Publish Joint states of robot (`#24 <https://github.com/snt-arg/unitree_ros/issues/24>`_)
* fix: joint indexes
* feat(config): add default topic name for joint states
* feat(driver): create function to retrive motor states
* feat: create serializer function to create join_state msg
* feat: create publisher for joint_state
* feat(ci): add a specific container image to each ci
* fix(ci/jazzy): add container with ubuntu noble
* docs: update location of build status
* chore(ci/jazzy): use relative version
* fix(ci/jazzy): use ros2-testing
* fix(ci/jazzy): remove v from the ros-setup action version
* fix(ci): bump setup-ros action to 0.7.7 which fixes a bug
* chore(ci): update workflow names and remove tests
* feat(ci): include ci for ros jazzy
* chore(ci): rename workflows
* chore(ci): upgrade to ros-tooling actions
* chore(ci): bump checkout actions to v4 and fix setup-ros distributions
* chore(ci): make use of ros-tooling
* docs: update usage section
* chore: add copyright notice to all source files
* Merge pull request `#28 <https://github.com/snt-arg/unitree_ros/issues/28>`_ from snt-arg/PedroS235-patch-1
  docs: remove foxy from being shown in status
* docs: remove foxy from being shown in status
* fix(docker): add missing dependency to be installed (`#23 <https://github.com/snt-arg/unitree_ros/issues/23>`_)
* Update issue and feature templates
* docs: update title
* docs: add missing > to the important alert
* docs: update usage instructions
* chore: update image
* Contributors: Pedro Soares

1.1.1 (2024-04-25)
------------------
* Merge pull request `#22 <https://github.com/snt-arg/unitree_ros/issues/22>`_ from snt-arg/dev
  fix: import error (`#21 <https://github.com/snt-arg/unitree_ros/issues/21>`_)
* fix: import error (`#21 <https://github.com/snt-arg/unitree_ros/issues/21>`_)
  This commit fixes issue (`#21 <https://github.com/snt-arg/unitree_ros/issues/21>`_) and changes the way we use the launch
  file. Now we only need use wifi:=true instead of passing the robot ip.
* chore: update gitignore to conform ros2 development
* Merge pull request `#20 <https://github.com/snt-arg/unitree_ros/issues/20>`_ from snt-arg/PedroS235-patch-1
  docs: update installation with apt
* docs: update installation with apt
* Contributors: Pedro Soares

1.1.0 (2024-03-25)
------------------
* fix: only make foxy CI run on the foxy branch
* chore: merge changes made in foxy branch
* build: fix deprecation warning
* build: add boost build dependency
* build: add missing depdendencies for tf2 in package.xml
* chore: change submodule url to https
* Merge pull request `#19 <https://github.com/snt-arg/unitree_ros/issues/19>`_ from snt-arg/PedroS235-patch-3
  docs: fix some miscellaneous typos
* docs: fix some miscellaneous typos
* Update README.md
* Merge pull request `#18 <https://github.com/snt-arg/unitree_ros/issues/18>`_ from snt-arg/PedroS235-patch-2
  chore: update license to GPL
* chore: update license to GPL
* Merge pull request `#17 <https://github.com/snt-arg/unitree_ros/issues/17>`_ from snt-arg/feature/license
  [Misc] Update license to GPLv3
* [Misc] Update license to GPLv3
* Merge pull request `#16 <https://github.com/snt-arg/unitree_ros/issues/16>`_ from snt-arg/PedroS235-patch-1
  docs(fix): specify the unitree go1 edu version
* docs(fix): specify the unitree go1 edu version
* Update README.md
* Contributors: Hriday Bavle, Pedro Soares

1.0.0 (2024-02-08)
------------------
* Update README.md
* fix(docs): TOC
* feat(docs): add emojis
* refactor(docs): clean description
* fix(docs): change info alert to note
* feat(docs): make documentation up to date
  * Improve description
  * Add new published topic `/sensor_ranges`
  * Update bms topic name
  * Correct dependencies
* Contributors: Pedro Soares

0.2.1 (2024-02-08)
------------------
* feat(launch): add robot_ip argument for easier change
* Contributors: Pedro Soares

0.2.0 (2024-02-08)
------------------
* fix(driver): quaternion has been corrected
* style(config): apply formatter
* refactor(docs): remove instructions for only foxy distro
* chore: update unitree sdk submodule (`#15 <https://github.com/snt-arg/unitree_ros/issues/15>`_)
* feat(docs): add build status for ros distros
* feat(actions): add workflow names
* feat(actions): move each distro build to separate workflows
* feat(ci): add humble and iron docker images to the pipeline
* feat(docker): create seperate images for distros humble and iron
* fix(docker): remove ssh keys from the image
* fix: add missing includes
* chore: update submodule to use snt-arg fork
* Contributors: Pedro Soares

0.1.9 (2024-02-02)
------------------

0.1.8 (2024-01-15)
------------------
* feat(driver): increase the receive high state rate to 2ms
* docs: update license and add a section comming soon
* chore: update licence to snt non commercial
* chore: bump version
* feat: update sdk to latest version
* build: add sensor ranges msg
* fix: fix quaternions and imu
* feat: add a serializer for sensor ranges
* refactor: change orientation to quarternion
* feat: increase the odometry publish rate
* refactor: clean the launch file
* fix: change base_link frame id to base_link
* [docs] Remove temporary command from installation guide
* [docs] Correct typos and remove emojis from titles
  With emojis on the titles, the hyperlinks were not working for some reason on GitHub.
* [build(fix)] Fix the problem of linking the faceLight library
* [docs] Add gifs demonstrating statuses LEDs
* Merge pull request `#14 <https://github.com/snt-arg/unitree_ros/issues/14>`_ from snt-arg/dev
  [Hotfix] Fix problem with the initial battery level check
* [docs] Fix toc links and added a required installation command
* Merge branch 'dev' of github.com:snt-arg/unitree_ros into dev
* [fix] Add a delay before readin battery level when starting
* Merge pull request `#13 <https://github.com/snt-arg/unitree_ros/issues/13>`_ from snt-arg/dev
  Tweeks to the driver and documentation
* [docs] Add emojis to titles
* [fix] Remove the rest of the unused message files
* [build] Remove unecessary message files
* [chore] Remove unecessary message files
* [refactor] Add some missing docstrings and removed dead code
* Merge branch 'dev' of github.com:snt-arg/unitree_ros into dev
* [chore] Change unitree_dat to common_defines
* [chore] Change unitree_data to common_defines
* [docs] Fix toc
* [docs] Improve ros topics section and add a paramers table
* [docs] Apply formatting
* [refactor] Remove unused ros timer and its callback function
* [docs] Fix typos
* Update README.md
  [docs] Fix the table of contents
* [docs] Add a table of contents and fixed some typos
* Merge pull request `#12 <https://github.com/snt-arg/unitree_ros/issues/12>`_ from snt-arg/dev
  Merge a stable version of development branch to main
* [docs] Improve the documentaion on its entirety
* [license] Add a BSD-3 license to the project
* [docs] Remove the "not yet functional" from the stand up/down topics
* [docs] Update the dependencies section and add a section on robot status
* [build] Add facelight lib
  For some reason I need to manually copy the lib file into the pc /lib folder.
  This will need to be fixed in the future. I am leaving it as it is for now.
* [feat] Make face LED blink yellow when battery is below 30%
* Merge pull request `#11 <https://github.com/snt-arg/unitree_ros/issues/11>`_ from snt-arg/dev
  Merge a stable version of the development branch
* [chore] Bump version to 0.1.7
* [refactor] Remove debugging prints
* [build] Add faceLight library support
* [chore] Remove lib folder from begin ignored
* Merge pull request `#10 <https://github.com/snt-arg/unitree_ros/issues/10>`_ from snt-arg/feature/add_face_light_status
  [feat] Show status colors in robot face light
* [feat] Show status colors in robot face light
  In this commit, we have a new functionality which is to give robot status
  using the robot's face RGB lights.
  With this commit, only 3 types of status are currently available:
  * READY: means the robot is ready to be operated (green color)
  * IDDLE: means the robot is in the standing down (white color)
  * MOVING: means the robot is currently in movement (blue color)
* [chore] Add the faceLightSDK package
* [fix] Change default robot ip to 192.168.123.161
* Merge branch 'main' into dev
* [fix] Make the checkout clone recursively submodules
* [fix] Change setup.bash to setup.sh
* [fix] Change source to .
* [fix] Fix Steps
* [fix] Change entry workdir to package path
* [fix] Fix the build pipeline
* [fix] Merge the colcon build with the source
* [fix] Fix identation
* [chore] Add a docker file containing the ros package
* [fix] Add image to the container
* [fix] Trying to fix action
* Merge branch 'main' of github.com:snt-arg/unitree_ros into main
* [chore] Add github action to build package
* [chore] Bump package version to 0.1.6
* [fix] Change driver attribute to a unique_ptr
  The ros parameters for the robot ip and the robot target port were never being used.
  Thus this commit makes now use of them.
* [fix] Pass constructor parameters to class attributes
* [refactor] Add the alternative ip as a comment
* [docs] Removed extra introduction section
* Merge branch 'dev' of github.com:snt-arg/unitree_ros into dev
* [refactor] Change robot ip to use ethernet's ip
* Merge pull request `#7 <https://github.com/snt-arg/unitree_ros/issues/7>`_ from snt-arg/dev
  Merge stable version of the development branch
* [chore] Bump the version to 0.1.5
* [refactor] Add comments and cleaned code
* [refactor] Remove code for turning on foot leds
* [refactor] Set obstacle avoidance to false as the default value
* [chore] Bump version to 0.1.3
* [feat] Add a flag to enable/disable robot's obstacle avoidance
* [feat] Add a battery watcher and shutdown in case below a threshold (`#4 <https://github.com/snt-arg/unitree_ros/issues/4>`_)
* [refactor] Remove bottom value from sensor_ranges has is non existant
* [feat] Make the driver aware if it is connected to the robot on start
* [feat] Implement a detector to see if the connection to the robot is established
* Merge pull request `#5 <https://github.com/snt-arg/unitree_ros/issues/5>`_ from snt-arg/dev
  Ability to stand up/down added
* [fix] Fix the problem when telling the robot to stand up/down
* [feat] Try using the wireless remote from high command
* [docs] Update README.md
* [fix] apply merge fix
* [refactor] change namespace to empty string
* [fix] make the publishers use a reliable QoS
* [fix] Fix some problems related to command vel
* [feat] Add subscribers for stand_up/down + retrieve the ranges comming from sensor
* [fix] Change queue depth from 10 to 1 for the velocity command sub
* Merge branch 'main' of github.com:snt-arg/unitree_ros into main
* Improving the overall package
  This commit brings some improvements to the package.
  ## What has changed:
  1. A driver class has been created, which acts as a middleware between ros and UNITREE_SDK.
  2. Brings additional features such as stand up, stand down, a way of choosing different modes etc.
  3. It allows to easily add new features to the package thanks to the separation between classes
  ## These new changes have not yet been tested on the real robot, thus need to be taken with precaution
* Contributors: Pedro Soares

0.1.0 (2023-05-13)
------------------
* Bump version to 0.1.0
* [feat] Add and odometry reset flag and the logic to reset it
* [feat] Implement a simple obstacle avoidance logic.
  For this simple implementaion, when a velocity command is received, the ranges of
  the front, left and right sensors are checked. If we are moving forward and an object is
  in front, the velocity command will then be ignored. Same principle is applied to the other directions.
* [feat] Add an obstacle avoidance flag
* [feat] Create a utils header
  Currently, this header file contains a function to check if one of the 3 distances passed are within a range of collision to the robot.
* [refactor] Change methods names to follow the file name
* [feat] Add obstacle_avoidance flag
* [misc] Apply a new convention for launch file name
* [fix] fix the odometry orientation
* [fix] Merge fix
* [feat] Add a new transormation between base_footprint and base_link
* [fix] Change frame ids to the correct names
* [refactor] Update odom child to os_sensor
* [refactor] Lowercase imu frameid
* [fix] Merge fix
* [reafactor] Improve the launch file
* [refactor] Change odom frame ids default values
* [refactor] Add the right values for transform between lidar and body
* [refactor] Change odom child frame id to base_link
* [fix] Fix odom orientation
* [refactor] Change body frame id to base_link
* [refactor] Change body frame id to base_link
* [refactor] use function from conversion header file
* [refactor] update the order of attributes declaration
* [feat] create method to generate the odometry tf transformation
* [feat] broadcast a transform between odom and body
* [style] Apply formatting
* [feat] Add a static transform between lidar and body
* [build] Add tf2 as a dependency
* [refactor] Improved the cmd_vel reset callback
* [feat] Add the params file as a launch argument
* [feat] Send an emtpy cmd_vel to robot if no cmd_vel was received within a timeout
* [feat+refactor] Add a childFrameId for the odometry and refactored the code
* [feat] Apply a timeout in the cmdvel callback in case no command is received to stop the robot
* [misc] Update submodule
* [misc] Update submodule
* [fix] Update branch to use v3.8.0
* [fix] Update branch to use v3.8.0
* [fix] Update branch to be v3.8.0 since v3.8.6 is broken
* [feat] Add bms state to be published to a topic /bms_state
* [feat] Add bms state topic name
* [refactor] Cleaned the code
* [refactor] Cleaned the code
* [feat] Create function to generate both the imu and odometry msg
* [feat] Add odometry and imu frame ids
* [fix] Fix problme with GetRecv
* [refactor] Change the UDP constructor call to another one
* [refactor] Initialized class attributes
* [fix] Fix the network ports
* [feat] Add callback methods, pubs, subs, timers and topic names
* [feat] Implementation of the declared methods
* [misc] File renamed to unitree_driver_ros
* [refactor] Update to the new name of the driver class
* [build] Update CMakelists to install config and launch folders + misc
* [refactor] applied formatting
* [feat] Create a launch file to execute the driver node
* [feat] Include some conversion functions from unitree repo
* [feat] Add some necessary parameters
* [docs] Update readme
* Delete .cache/clangd/index directory
* [docs] Update the introduction
* [feat] Create the basic private attributes for the driver
* [build] Add the required dependencies and ros messages
* [feat] Imported the necessary ros messages from unitree_ros_to_real
* [misc] Renamed file to unitree_driver_ros.hpp
* [feat] Create a config file for ros parameters
* [refactor] Update gitignore
* [refactor] Update gitignore
* [refactor] Update indetation to 4
* Update README.md
* [Feat] Add unitree sdk as submoduel
* [build] Adding unitree_legged_sdk to the CMakelists
* [feat] Create a simple ROS node
* [misc] Create a clang-format file
* [misc] Update package.xml description
* [misc] Create empty ros2 package
* Initial commit
* Contributors: Hriday Bavle, Pedro Soares, hriday
