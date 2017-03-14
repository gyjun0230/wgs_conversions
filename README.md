# wgs_conversions #
A ROS server implementation of the World Geodetic System (WGS) conversion functions for converting to/from LLA, ENU, and ECEF.

## Available Services ##
### xyz2lla ###
Converts from ECEF coordinates (in meters) to Latitude, Longitude, Altitude (in degrees).

### lla2xyz ###
Converts from Latitude, Longitude, Altitude (in degrees) to ECEF coordinates (in meters).

### lla2enu ###
Converts from Latitude, Longitude, Altitude (in degrees) to a local level frame of the East, North, Up convention (in meters).

Note: This requires a reference LLA position to serve as the origin of the local ENU frame. 

### enu2lla ###
Converts from a local East, North, Up frame (in meters) to Latitude, Longitude, Altitude (in degrees).

Note: This requires a reference LLA position to serve as the origin of the local ENU frame. 

### xyz2enu ###
Converts from ECEF coordinates (in meters) to East, North, Up (in meters).

Note: This requires a reference LLA position to serve as the origin of the local ENU frame. 

### enu2xyz ###
Converts from East, North, Up (in meters) to ECEF (in meters).

Note: This requires a reference LLA position to serve as the origin of the local ENU frame. 

## Usage ##
To familiarize yourself with using the ROS services, see the code in the example directory. This directory contains examples of a C++ and a python client.

From the workspace directory, run catkin_make with BUILD_EXAMPLE option set to ON
```
catkin_make -DBUILD_EXAMPLE=ON
```

Run the ROS server for WGS conversions
```
rosrun wgs_conversions wgs_conversions_server
```

In a new terminal, run the C++ example client
```
rosrun wgs_conversions wgs_conversions_client
```

Run the python example client
```
rosrun wgs_conversions wgs_conversions_client.py
```

## References ##
Decker, B. L., World Geodetic System 1984, Defense Mapping Agency Aerospace Center. 

## Author ##
Andrew Barrows (original MATLAB implementation). 
Dan Pierce (ROS/C++ implementation)

## Maintainer ##
Dan Pierce (danpierce,jdp0009@auburn.edu)