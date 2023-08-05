# robot_mimicker
A Master-Slave controller using OpenManuiplator-X

Robot Mimicker ROS Package
==========================

This repository contains a ROS package named **robot\_mimicker\_pkg** that provides functionality for mimicking robot actions. The package includes a launch file, a Python script, and configurations necessary for its proper operation.

Cloning the Repository
----------------------

To clone the **robot\_mimicker\_pkg** repository to your `catkin_ws` directory, follow these steps:

1.  Open a terminal window.
    
2.  Navigate to your `catkin_ws/src` directory:
    
    bashCopy code
    
    `cd ~/catkin_ws/src`
    
3.  Clone the repository using `git`:
    
    bashCopy code
    
    `git clone https://github.com/ghareeb97/robot_mimicker_pkg.git`
    

Building the Package
--------------------

After cloning the repository, navigate to your `catkin_ws` directory and build the package:

1.  Navigate to your `catkin_ws` directory:
    
    bashCopy code
    
    `cd ~/catkin_ws`
    
2.  Build the workspace:
    
    bashCopy code
    
    `catkin_make`
    
3.  Source the workspace:
    
    bashCopy code
    
    `source ~/catkin_ws/devel/setup.bash`
    

Package Structure
-----------------

The repository's structure is as follows:

cssCopy code

`catkin_ws/ ├── src/ │   ├── robot_mimicker_pkg/ │   │   ├── launch/ │   │   │   └── robot_mimic.launch │   │   ├── scripts/ │   │   │   └── robot_mimicker.py │   │   └── CMakeLists.txt └── README.md`

Launching the Package
---------------------

To launch the **robot\_mimic.launch** file, which loads various components and configurations, follow these steps:

1.  Open a terminal window.
    
2.  Navigate to your `catkin_ws` directory:
    
    bashCopy code
    
    `cd ~/catkin_ws`
    
3.  Source your ROS environment if you haven't already:
    
    bashCopy code
    
    `source ~/catkin_ws/devel/setup.bash`
    
4.  Launch the **robot\_mimic.launch** file using the `roslaunch` command:
    
    bashCopy code
    
    `roslaunch robot_mimicker_pkg robot_mimic.launch`
    

This will initiate the launching process, which includes setting up controllers, loading configurations, and starting the **robot\_mimicker.py** node.

Description of Launch Configuration
-----------------------------------

The **robot\_mimic.launch** file contains configurations for launching different components:

*   **Master Group**: This section launches components related to the master robot.
*   **Slave Group**: This section launches components related to the slave robot.
*   **robot\_mimicker.py Node**: This section launches the Python script named **robot\_mimicker.py**.

Please note that the launch file is set up to mimic specific hardware and controller configurations. You might need to modify the launch file and configurations according to your own hardware setup.

About robot\_mimicker.py
------------------------

The **robot\_mimicker.py** script is located in the `scripts/` directory. It provides functionality for mimicking robot actions. You can modify and extend this script as needed for your specific use case.
