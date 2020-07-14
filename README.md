# AutoZ-2020
* CmakeList.txt is added to gitignore. So Clone it directly.
* Then do a Catkin_make

## Files
### autoz_nav
> Contains Work done by GLL and PPL teams
### differntial_drive
> Master File Contains Everything from the beginning along with the modifications made in 2020
### autoz_urdf
> Used to check and Spawn URDF model in Gazebo. To check your URDF, place your URDF file in autoz_urdf/urdf and your URDF file name should be "myrobot1.urdf".
> Use rosrun gazebo_ros then launch autoz_urdf.launch in autoz_urdf/launch
### r2d2
> It is the robot_state_publisher package and the final URDF should be in r2d2/urdf directory
### Notes
* All the Chefbot files not original chefbot files, they are used to test our bot and modified for test purposes
* All TurtleBot Files are Original TurtleBot Files
* Global Localization Package("robot_localization") has been removed for error purpose and tweaking parameters for now.
* And while running the main launch file ("myrobotdrive.launch") it may show some error like "No such Directory". Which every place it shows, change that directory name from other name to your local machine name.
