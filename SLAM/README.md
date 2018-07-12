
This project implements Particle Filter SLAM with six initial probable locations as defined in "map.PNG".
For more details about implementation and execution, please refer report.pdf.

Requirements:

    Ubuntu 16.04
    ROS Kinetic
    Simulation: Gazebo
    Hardware: Pioneer P3-DX
    Packages: p20s_msgs

Running the project on Hardware.

    - Change the path of the map in mapGUI.py and safegoto.py
	- Run localization.py and safegoto.py
	  Safegoto.py starts execution after localization.py pubishes the localized pose.
