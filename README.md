Swarmathon-Arduino
==============

This repository is an Arduino microcontroller library for the Swarmie robots used in the [NASA Swarmathon](http://www.nasaswarmathon.com), a national swarm robotics competition created by the [University of New Mexico](http://www.unm.edu/). This particular library is an interface to the [Swarmathon-ROS](https://github.com/BCLab-UNM/Swarmathon-ROS) controller framework that facilitates the opertion of lower-level functionality onboard the physical robot, including brushed DC motors, integrated quadrature encoders, a 10-axis IMU (inertial measurement unit), and ultrasonic distance sensors.

For information on setting up the Arduino IDE and programming Arduino microcontrollers, please consult [Getting Started with Arduino](https://www.arduino.cc/en/Guide/HomePage). Aside from the IDE, no other plugins or tools are required to begin using this library.

After installing the Arduino IDE, run the application and open the Arduino IDE Preferences window (under "File > Preferences" in Linux and Windows, or "Arduino > Preferences" in Mac OS X). Under the Settings tab, in the text box titled "Sketchbook location", enter the full path to your Swarmathon-Arduino directory, then click "OK":

![Alt text](http://swarmathon.cs.unm.edu/img/Sketchbook.png "Arduino IDE Sketchbook location")

**Note** that you must exist and reopen the Arduino IDE before the change to the Sketchbook location is applied.
