me_scancontrol_b
==========
## Installation Instructions:
- install aravis0.6.0
- run 'install.sh /path/to/C++ SDK (Linux)/' to install me header files and libraries
- edit launch/scanner.launch to fit your need. 'path_to_device_properties' must point to a directory in which 'device_properties.dat' is located.

## Launch file parameters:
| Parameter  | Description |
| ------------- | ------------- |
| shutter_time  | Duration during which the shutter is open. Unit is 10 microseconds.  |
| idle_time  | Duration from last close of shutter to next open of shutter. Unit is 10 microseconds. Freuqency in Hz of scan is determined by 100000/(shutter_time+idle_time) |
| container_size  | Size of profile container. Sets the number of profiles which are collected by scanner and delivered together to PC.  |
| frame  | The frame of the scanner. Has to be a valid tf frame and should be set at the middle of the emission of the laser  |
| path_to_device_properties(optional)  | directory in which 'device_properties.dat' is located. Default is /opt/scanCONTROL/  |
| topic (optional)  | ROS-topic to which each scanned profile is published. Default is 'laser_scan'  |
| serial_number (optional)  | Serial Number of the scanner. Used if more than one scanner is used in the same network. If left blank, connects to first found scanner. |
| field_left (optional)  | How much should meassurement field be restricted on the left side. Unit is % of whole field. Default is 0. |
| field_right (optional)  | How much should meassurement field be restricted on the right side. Unit is % of whole field. Default is 0.  |
| field_far (optional)  | How much should meassurement field be restricted on the far side. Unit is % of whole field. Default is 0.  |
| field_near (optional)  | How much should meassurement field be restricted on the near side. Unit is % of whole field. Default is 0. |
| lag_compensation (optional)  | Offset to eliminate constant network lag between scanner and PC. Unit is seconds. Default is 0-  |

## Notes:
- Currently uses fixed profile size of 640
- Tested with kinetic
