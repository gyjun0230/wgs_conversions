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

### xyz2enu_vel ###
Converts velocities from ECEF coordinates (in meters/sec) to East, North, Up (in meters/sec).

Note: This requires a reference LLA position to serve as the origin of the local ENU frame. 

### enu2xyz_vel ###
Converts velocities from East, North, Up (in meters/sec) to ECEF (in meters/sec).

Note: This requires a reference LLA position to serve as the origin of the local ENU frame. 

### xyz2enu_cov ###
Converts covariance matrix (of both position and velocity) from ECEF coordinates to East, North, Up.

Note: This requires a reference LLA position to serve as the origin of the local ENU frame. 

### enu2xyz_cov ###
Converts covariance matrix (of both position and velocity) from East, North, Up to ECEF.

Note(1): This requires a reference LLA position to serve as the origin of the local ENU frame.

Note(2): If you are working in North, East, Down (NED) convention and want to convert an ECEF covariance into NED, the enu2xyz_cov service can still be helpful. Just rotate the ENU covariance to NED using

```
P_ned = R P_enu R'
```
where P_enu is the ENU covariance (returned from enu2xyz_cov), P_ned is the NED covariance, R is a 3x3 matrix that represents the transformation from ENU to NED (shown below), and R' is the transpose of R.

```
R = [0  1  0]
    [1  0  0]
    [0  0 -1]
```

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

## To do ##
- Add UTM conversions
- Allow for NED local frame instead of ENU
- Reference position as a class variable

## References ##
Decker, B. L., World Geodetic System 1984, Defense Mapping Agency Aerospace Center. 

## Author ##
Andrew Barrows (original MATLAB implementation). 
Dan Pierce (ROS/C++ implementation)

## Maintainer ##
Dan Pierce (danpierce,jdp0009@auburn.edu)
