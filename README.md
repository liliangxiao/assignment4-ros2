Ryder does:
1. Used Assignment3 is workspace
2. Integrated codes in:
3. assignment4-ros2/assignment4_WS/src/tb3_autonomous/tb3_autonomous/references
    square_mover for exploring, scan_and_move for obstacle avoidance, assignment 2 code revised for marker detection.
4. Commented in new_world.launch.py for easy testing:
5.     #ld.add_action(vision_nav), you need to uncomment this line before sumbition.
       you need manually run the main script to start our logic
6. Revised the world file to integrate arcuro markers for detection.

Commands to Run:
1. Buid:
ryder@ryder-Aspire-A315-24P:~/Documents/as4/assignment4-ros2/assignment4_WS$ colcon build
Starting >>> tb3_autonomous
[0.912s] WARNING:colcon.colcon_ros.task.ament_python.build:Package 'tb3_autonomous' doesn't explicitly install a marker in the package index (colcon-ros currently does it implicitly but that fallback will be removed in the future)
Finished <<< tb3_autonomous [1.04s]          
Summary: 1 package finished [1.42s]

2. Install
ryder@ryder-Aspire-A315-24P:~/Documents/as4/assignment4-ros2/assignment4_WS$ source install/setup.sh 

3. Run new_world:
ryder@ryder-Aspire-A315-24P:~/Documents/as4/assignment4-ros2/assignment4_WS$ ros2 launch tb3_autonomous new_world.launch.py

4. Run Main Script In New command line:
Setup first:
ryder@ryder-Aspire-A315-24P:~/Documents/as4/assignment4-ros2/assignment4_WS$ source install/setup.sh
Run main script:
ryder@ryder-Aspire-A315-24P:~/Documents/as4/assignment4-ros2/assignment4_WS$ python3 src/tb3_autonomous/tb3_autonomous/CameraSlam.py

TODO:
1. Debug the integrated file to detect arcuro markes and avoid obstacles.
2. ADD SLAM function
3. report

