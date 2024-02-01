# Leuze ROS2 drivers

This stack contains all packages of the ROS2 driver for the Leuze RSL 400 laser scanners.


## Installation (from source)

Activate the ROS2 Humble installation with:
```
$ source /opt/ros/humble/setup.bash
```
Note: You need to repeat this command for each new terminal.


Navigate to the src subfolder of your colcon workspace:
```
$ cd ~ros2_ws/src/
```

Copy the complete source code of this driver into the current folder, for example by cloning it from github
```
$ git clone https://github.com/thesensorpeople/leuze_rsl_ros2_drivers.git
```

Navigate to the main workspace folder:
```
$ cd ..
```

Install all ROS2 dependencies with rosdep:
```
rosdep install --from-paths src --ignore-src -r -y
```

Build the driver
```
$ colcon build --symlink-install
```

Next, you need to source this workspace so that ROS2 can see the new packages:
```
$ source install/local_setup.bash
```


Alternatively, you can build only the single packages required by your project, for example:
```
colcon build --symlink-install --packages-select leuze_msgs
source install/local_setup.bash

colcon build --symlink-install --packages-select leuze_rsl_driver
source install/local_setup.bash

colcon build --symlink-install --packages-select leuze_bringup
source install/local_setup.bash
```

## Simulation mode
You can activate the simulation mode for testing this driver without any connected real laser scanner by switching the option "SIMULATION" in leuze_rsl_driver/CMakeLists.txt from "no" to "yes":
```
#------------------SIMULATION---------------------------
set(SIMULATION "yes")     #Set to "yes" for simulation mode (default value: "no")
```

In rviz, the simulated laser scanner will appear as dynamic sinusoidial contour within the defined topic name.


## Scanner Setup

You can visit [this page](https://www.leuze.com/en/deutschland/produkte/produkte_fuer_die_arbeitssicherheit/optoelektronische_sicherheits_sensoren/sicherheits_laserscanner/rsl_4_5/selector.php?supplier_aid=53800144&grp_id=1411040450707&lang=eng#{%22tab%22:%223%22}) on the Leuze official website to download the *Sensor Studio* software tool and the *Quick Start Guide* document to help setup the communication settings of the scanner. Follow the instructions from the guide until you are successfully connected to the scanner.   

Now we need to setup the static IP address of the scanner as well as the receiving device. To do so, go to the *Settings* tab on the top and expand the *Communication* option to the left.   

![Alt text](leuze_description/doc/SensorStudio_IP1.PNG?raw=true "IP Settings")

Enter your desired static address for the scanner in the `IP address` field (this is the value you provide to the launch file as described in section *Bringup*) and the subnet mask in the `Subnet mask` field. The addresses assumed by default in this driver stack are `192.168.10.1` and `255.255.255.0` respectively.

Next, select the *Data telegrams* option to the left within the same tab. This allows us to setup the various settings for the UDP telegrams.   

![Alt text](leuze_description/doc/SensorStudio_IP2.PNG?raw=true "UDP Settings")

The various settings are :
* `UDP Telegram` : Ensure this is set to `Active` so we can actually receive data in the driver.   
* `Destination` : Should be set to `IP address`.   
* `IP address` : This refers to the IP address of the device receiving the datagrams (i.e. the device running this driver stack). You can set this to any desired value, the default assumed by this stack is `192.168.10.2`.  
* `Device name` : Enter any name you wish.   
* `Port` : You can enter any value you wish. The default assumed by this driver stack is `9990`.   
* `Measurement value transmission` : Ensure set to `Active`.   
* `Data type` : This allows you to select between `ID: 6` for Distance only or `ID: 3` for Distance+Signal Strength. Both are supported by this driver.
* The next 3 fields allow you to setup the scan area and resolution. Once you set these values, make sure to also update them in `leuze_rsl_driver/config/params.yaml` as well (albeit converted to radians). Failing to update the values only shows a warning during execution, but does not impair functionality. The default values can be seen in the image above and the *yaml* file respectively.


Once these settings have been updated, they need to be written to the device. You can do so by clicking on the small blue down arrow button at the top. Only the communication settings need to be updated, so select only this parameter in the ensuing dialog box. To upload any settings, you need to use the `Engineer` profile, and the password is `safety` be default. **PLEASE BE SURE OF ANY SETTINGS YOU UPLOAD THIS WAY.** Once you change the IPs, you may need to restart the scanner as well as the receiving device to reconnect.

## Receving device Setup

Once the IPs have been setup correctly on the scanner, setting up the receiving device is relatively straightforward. You only need to create a new Wired connection with a desired name. The IPv4 address should be the IP address you entered in *Settings>Data telegrams>IP address* in Sensor Studio, the default value assumed by this driver stack is `192.168.10.2`. The subnet mask would then similarly be `255.255.255.0` as previously setup in *Settings>Communication>LAN*.

## Phidget driver (obsolete - will be removed in future versions)

This driver stack contains a Phidget driver in order to interface with the I/Os of the scanner. A required Phidget Interface Kit can be found [here](https://github.com/ros-drivers/phidgets_drivers). If you wish to utilize this feature, either install the Phidget driver from source by cloning it to the same workspace and building it, or install it directly as a Debian binary package:   

```
sudo apt install ros-<distro>-phidgets-ik
```
where `<distro>` is your distribution of ROS *(Kinetic/Melodic)*.   

You can make sure you have everything else you need by running the following from your workspace directory:   
```
rosdep install --from-paths src --ignore-src -r -y
```    

## Packages description
- `leuze_bringup` : Contains the launch files for starting the ROS driver, main point of entry to this stack.   
- `leuze_description` : Contains the URDF of the scanner and the launch file for independently viewing it.   
- `leuze_msgs` : Contains all the custom messages required for internal functionality.  
- `leuze_phidget_driver` : Contains the Phidget IK driver package to read the I/Os of the scanner.   
- `leuze_ros_drivers` : Metapackage of this stack.   
- `leuze_rsl_driver` : Contains the main driver source code and its tests.   

## Bringup
You can start the Leuze RSL ROS driver for RSL400 by running :
```
ros2 launch leuze_bringup leuze_bringup_rsl400.launch.py sensor_ip:=<sensor ip> port:=<port> topic:=<topic name>
```

By using different topic names, you can lauch multiple driver instances for multiple connected scanners at a time:

Example:
```
ros2 launch leuze_bringup leuze_bringup_rsl400.launch.py sensor_ip:=192.168.20.5 port:=9991 topic:=scan1
ros2 launch leuze_bringup leuze_bringup_rsl400.launch.py sensor_ip:=192.168.20.7 port:=9992 topic:=scan2
```

#### Parameters
 - `sensor_ip` : The IPv4 address of the laser scanner. This can be configured from the Sensor Studio software tool in Windows. The scanner also displays its currently configured IP during power on startup.   
- `port`: The port number of the scanner. Can be similarly configured on Sensor Studio, but not displayed on the sensor during startup.
- `topic`: The topic name under which the driver appears in the Rviz tool 

> For more information on how to setup the IP and Port values of the scanner using Sensor Studio, see section *Scanner Setup*. 

Further parameters are defined in the corresponding .YAML files:

leuze_bringup/config/params_rsl400.yaml

These additional parameters are:
    `scan_size`:  Number of beams in a single scann
    `angle_min`:  Lowest possible angle: -135° (-3*pi/4 rad)
    `angle_max`:  Highest possible angle: +135° (+3*pi/4 rad)
    `scan_time`:  Period of the laser scanner
    `range_min`:  Minumum measurable distance
    `range_max`:  Maxumum measurable distance
    `scan_frame`: Laser scan frame ID for further transformations

*Note*
If you only want rviz to visualize the measured contour without defining any additional transformation, you must set the Fixed_frame property in rviz to the exact name of the Laser scan frame ID (scan_frame) defined above.

You can view the scan profile by running :   
```
ros2 launch leuze_description view_rsl400.launch.py
```


## Unit test and code verification
Use colcon to perform code verification (cpplint, uncrustify) and to run unit tests:
```
colcon test
colcon test-result --all
```