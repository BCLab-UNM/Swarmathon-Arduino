Swarmathon-Arduino
==============

This repository is an Arduino microcontroller library for the Swarmie robots used in the [NASA Swarmathon](http://www.nasaswarmathon.com), a national swarm robotics competition created by the [University of New Mexico](http://www.unm.edu/). This particular library is an interface to the [Swarmathon-ROS](https://github.com/BCLab-UNM/Swarmathon-ROS) controller framework that facilitates the opertion of lower-level functionality onboard the physical robot, including brushed DC motors, integrated quadrature encoders, a 10-axis IMU (inertial measurement unit), and ultrasonic distance sensors.

For information on setting up the Arduino IDE and programming Arduino microcontrollers, please consult [Getting Started with Arduino](https://www.arduino.cc/en/Guide/HomePage). Aside from the IDE, no other plugins or tools are required to begin using this library.

If you are using Linux, you'll need to run several additional commands to ensure that the Arduino IDE has the correct permissions to run. First, open a **Terminal** window and run the command `sudo usermod -a -G dialout username` to add your user account to the `dialout` user group, where `username` should be replaced by your own user name. Then, run the command `echo 'ATTRS{idVendor}=="1ffb", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/77-arduino.rules` to force Ubuntu's modem-manager to ignore the Arduino when it is plugged in. Finally, reload the `udev` device manager with the command `sudo udevadm trigger`.

After installing the Arduino IDE, run the application and open the Arduino IDE Preferences window (under "File > Preferences" in Linux and Windows, or "Arduino > Preferences" in Mac OS X). Under the Settings tab, in the text box titled "Sketchbook location", enter the full path to your Swarmathon-Arduino directory, then click "OK":

![Arduino IDE Sketchbook location](http://swarmathon.cs.unm.edu/img/Sketchbook.png)

**Note** that you must exit and reopen the Arduino IDE before the change to the Sketchbook location is applied.

## Setup

1. To set up the Arduino IDE to communicate with the Swarmie's [Pololu A-Star microcontroller](https://www.pololu.com/product/3104), which runs an Arduino-compatible bootloader, first set the board type under "Select > Board" to "Arduino Leonardo".

  ![Arduino IDE Board Type](http://swarmathon.cs.unm.edu/img/ArduinoIDEBoardType.png)

2. Ensure that the A-Star is plugged into your PC (not the Swarmie's NUC), then select the proper serial port under "Select > Port". Your port number will most likely differ from the one shown in the screenshot below, but you should still see "Arduino Leonardo" next to the correct port.

  ![Arduino IDE Serial Port](http://swarmathon.cs.unm.edu/img/ArduinoIDESerialPort.png)

3. If you haven't loaded it already, open the Swarmathon_Arduino.ino sketch under "File > Open" by navigating to your Swarmathon-Arduino directory.

  ![Arduino IDE Open Sketch](http://swarmathon.cs.unm.edu/img/ArduinoIDEOpenSketch.png)
  ![Arduino IDE Open Sketch](http://swarmathon.cs.unm.edu/img/ArduinoIDEOpenSketch2.png)

4. Upload the sketch to the A-Star by clicking on the "Upload" button, a right arrow in the upper-left corner of the Arduino IDE.

  ![Arduino IDE Upload Sketch](http://swarmathon.cs.unm.edu/img/ArduinoIDEUploadSketch.png)


## Debugging

