#!/usr/bin/env python
"""
An example client for using the wgs_conversions services

Dan Pierce
2017-03-13
"""
import sys
import rospy
from wgs_conversions.srv import WgsConversion

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert from XYZ to LLA
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
x = 422143.588316416
y = -5361864.950247487
z = 3417050.753609172

print "\nConverting position"
print "\tX: ",x,"\tY: ",y,"\tZ: ",z

rospy.wait_for_service('xyz2lla')

xyz2lla = rospy.ServiceProxy('xyz2lla', WgsConversion)

rsp = xyz2lla(xyz=(x,y,z))
(lat,lon,alt)=rsp.lla

print "\n\tSuccessfully converted from ecef to lla"
print "\t\tLat: ",lat,"\tLong: ",lon,"\tAlt: ",alt

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert from LLA to XYZ
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
rospy.wait_for_service('lla2xyz')

lla2xyz = rospy.ServiceProxy('lla2xyz', WgsConversion)

rsp = lla2xyz(lla=(lat,lon,alt))
(x,y,z)=rsp.xyz

print "\n\tSuccessfully converted from lla back to ecef"
print "\t\tX: ",x,"\tY: ",y,"\tZ: ",z

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert from LLA to ENU
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
ref_lat = lat + 0.01
ref_lon = lon + 0.01
ref_alt = alt + 10.0

rospy.wait_for_service('lla2enu')

lla2enu = rospy.ServiceProxy('lla2enu', WgsConversion)

rsp = lla2enu(lla=(lat,lon,alt),ref_lla=(ref_lat,ref_lon,ref_alt))
(e,n,u)=rsp.enu

print "\n\tSuccessfully converted from lla to enu"
print "\t\tE: ",e,"\tN: ",n,"\tU: ",u

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert from ENU to LLA
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
rospy.wait_for_service('enu2lla')

enu2lla = rospy.ServiceProxy('enu2lla', WgsConversion)

rsp = enu2lla(enu=(e,n,u),ref_lla=(ref_lat,ref_lon,ref_alt))
(lat,lon,alt)=rsp.lla

print "\n\tSuccessfully converted from enu back to lla"
print "\t\tLat: ",lat,"\tLong: ",lon,"\tAlt: ",alt

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert from XYZ to ENU
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
rospy.wait_for_service('xyz2enu')

xyz2enu = rospy.ServiceProxy('xyz2enu', WgsConversion)

rsp = xyz2enu(xyz=(x,y,z),ref_lla=(ref_lat,ref_lon,ref_alt))
(e,n,u)=rsp.enu

print "\n\tSuccessfully converted from xyz to enu"
print "\t\tE: ",e,"\tN: ",n,"\tU: ",u

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert from ENU to XYZ
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
rospy.wait_for_service('enu2xyz')

enu2xyz = rospy.ServiceProxy('enu2xyz', WgsConversion)

rsp = enu2xyz(enu=(e,n,u),ref_lla=(ref_lat,ref_lon,ref_alt))
(x,y,z)=rsp.xyz

print "\n\tSuccessfully converted from enu back to xyz"
print "\t\tX: ",x,"\tY: ",y,"\tZ: ",z

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert velocities from XYZ to ENU
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
vx = 23.3883
vy = 2.1984
vz = 0.4462

print "\nConverting velocity"
print "\tvX: ",vx,"\tvY: ",vy,"\tvZ: ",vz

rospy.wait_for_service('xyz2enu_vel')

xyz2enu_vel = rospy.ServiceProxy('xyz2enu_vel', WgsConversion)

rsp = xyz2enu_vel(xyz=(vx,vy,vz),ref_lla=(ref_lat,ref_lon,ref_alt))
(ve,vn,vu)=rsp.enu

print "\n\tSuccessfully converted velocities from xyz to enu"
print "\t\tvE: ",ve,"\tvN: ",vn,"\tvU: ",vu

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert velocities from ENU to XYZ
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
rospy.wait_for_service('enu2xyz_vel')

enu2xyz_vel = rospy.ServiceProxy('enu2xyz_vel', WgsConversion)

rsp = enu2xyz_vel(enu=(ve,vn,vu),ref_lla=(ref_lat,ref_lon,ref_alt))
(vx,vy,vz)=rsp.xyz

print "\n\tSuccessfully converted velocities from enu back to xyz"
print "\t\tvX: ",vx,"\tvY: ",vy,"\tvZ: ",vz

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert covariance from XYZ to ENU
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Pxyz =  ( (0.1527,-0.0244,0.0267),
	      (-0.0244,0.5772,-0.2432),
	      (0.0267,-0.2432,0.2687) )

print "\nConverting covariance"
print "\tPxyz:\t",Pxyz[0][0],"  ",Pxyz[0][1],"  ",Pxyz[0][2]
print "\t     \t",Pxyz[1][0],"  ",Pxyz[1][1],"  ",Pxyz[1][2]
print "\t     \t",Pxyz[2][0],"  ",Pxyz[2][1],"  ",Pxyz[2][2]

pxyz = []
for i in range(3):
	for j in range(3):
		pxyz.append(Pxyz[i][j])

rospy.wait_for_service('xyz2enu_cov')

xyz2enu_cov = rospy.ServiceProxy('xyz2enu_cov', WgsConversion)

rsp = xyz2enu_cov(xyz_cov=pxyz,ref_lla=(ref_lat,ref_lon,ref_alt))

Penu = ( (rsp.enu_cov[0],rsp.enu_cov[1],rsp.enu_cov[2]),
	     (rsp.enu_cov[3],rsp.enu_cov[4],rsp.enu_cov[5]),
	     (rsp.enu_cov[6],rsp.enu_cov[7],rsp.enu_cov[8]) )

print "\n\tSuccessfully converted covariance from ecef to enu"
print "\t\tPenu:\t",Penu[0][0],"  ",Penu[0][1],"  ",Penu[0][2]
print "\t\t     \t",Penu[1][0],"  ",Penu[1][1],"  ",Penu[1][2]
print "\t\t     \t",Penu[2][0],"  ",Penu[2][1],"  ",Penu[2][2]

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
"" Convert covariance from ENU to XYZ
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
penu = []
for i in range(3):
	for j in range(3):
		penu.append(Penu[i][j])

rospy.wait_for_service('enu2xyz_cov')

enu2xyz_cov = rospy.ServiceProxy('enu2xyz_cov', WgsConversion)

rsp = enu2xyz_cov(enu_cov=penu,ref_lla=(ref_lat,ref_lon,ref_alt))

Pxyz = ( (rsp.xyz_cov[0],rsp.xyz_cov[1],rsp.xyz_cov[2]),
	     (rsp.xyz_cov[3],rsp.xyz_cov[4],rsp.xyz_cov[5]),
	     (rsp.xyz_cov[6],rsp.xyz_cov[7],rsp.xyz_cov[8]) )

print "\n\tSuccessfully converted covariance from enu back to ecef"
print "\t\tPxyz:\t",Pxyz[0][0],"  ",Pxyz[0][1],"  ",Pxyz[0][2]
print "\t\t     \t",Pxyz[1][0],"  ",Pxyz[1][1],"  ",Pxyz[1][2]
print "\t\t     \t",Pxyz[2][0],"  ",Pxyz[2][1],"  ",Pxyz[2][2]