# Localization algorithm using mr-EKF for Elisa3 robots

This code-base is an improved version of the original code, which you can find [here](https://github.com/kemosabe564/ting_msc/blob/master/elisa3_node_cpp/README.md). The current code-base uses ROS and C++, and was developed on a Linux (Ubuntu 20.04 LTS) distribution and ROS Noetic. 

## How to use Elisa3 robots

Please follow the instructions from the [GCtronic](https://www.gctronic.com/doc/index.php/Elisa-3) website to find all the information related to testing and calibrating the Elisa3 robots.

Some important sections:
- Navigate to the [software](https://www.gctronic.com/doc/index.php/Elisa-3#Software) section to upload the advanced demo firmware to the Elisa3 robots. In the same section, you can find the Elisa3_remote_monitor in (section 4.3) to test the robots in an interactive GUI
- Navigate to the [odometry](https://www.gctronic.com/doc/index.php/Elisa-3#Odometry) section to calibrate the robots for odometry
- Navigate to the [ROS](https://www.gctronic.com/doc/index.php/Elisa-3#ROS) section to find the ROS-based Elisa3 code

Please do test the robots to figure out which of them work properly. Some of these robots are faulty, and suffer from motor speed discrepancy (both the motors do not operate at the same power for a specific input speed)

## How to use the Motion Capture or OptiTrack system

Please find the PDF document named [Optitrack](Optitrack.pdf) in the root folder to find the related code and test the Optitrack system. To connect to the Optitrack system to your PC
- Connect the ethernet cable to your PC
- Make sure that the "Multicast address" in the "Data Streaming" panel on the Motive software matches the IP address of the wired connection on your PC.
To calibrate the system, follow the instrucitons in this [video](https://www.youtube.com/watch?v=aK1cpr6ShPE).

## How to use this repository

Clone the repository into the catkin workspace folder `catkin_ws/src` (or the name of your workspace). The `elisa3_node_cpp` and `mocap_optitrack` folders are already present in the source folder. Install the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page), [JSON](https://github.com/nlohmann/json), and [Gnuplot](http://www.gnuplot.info/) libraries as they are used in the code. The `conversion_to_cpp_v2` branch contains some old functions that are not used. Feel free to refer to them if necessary. 

Before you run the code, place the robots on the field and edit the robots' address and intial position (obtained from the Optitrack system) in the files [robots.yaml](elisa3_node_cpp/config/robots.yaml), [mapper.json](estimator/src/mapper.json), and [mocap.yaml](mocap_optitrack/config/mocap.yaml). To see the live positioning of the robots on the Motive application, click on "View > Project". Do not forget to use the robot cap with markers before placing it in the field. 

To run the code, first navigate to `~catkin_ws/src/elisa3_node_cpp/src/pc-side-elisa3-library/linux`. Then run the commands (from terminal)
```
make clean
make
```

 You can then run the following commands from `catkin_ws` in the terminal:
```
rm -rf source devel
catkin_make
source devel/setup.bash
```

Open three additional windows pointing at `catkin_ws` and run the command `source devel/setup.bash` in all three windows. Then, run the following commands from the specified window

First window:
```
roscore
```
Second window:
```
roslaunch mocap_optitrack mocap_multidrone.launch
```

Third window:
```
roslaunch elisa3_node_cpp elisa3_swarm.launch
```

Fourth window:
```
rosrun estimator estimator
```

Currently, the code runs a consensus algorithm for the robots. You can modify this to run any other algorithm in the [estimator](estimator) node. The folder [data](estimator/data) stores the data from the experiment, and the folder [plots](estimator/plots) can be used to plot the data.

You can find my internship report [here](Shashank_Internship_Report_Final.pdf), and the thesis work of the original contributor to this code [here](https://repository.tudelft.nl/islandora/object/uuid%3A5ffff5d8-d1c8-4513-92c4-881f4b344558?collection=education). In case of any further questions, feel free to send an [email](mailto:shashank0911@hotmail.com). Cheers!
