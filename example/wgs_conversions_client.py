#!/usr/bin/env python
"""
An example client for using the wgs_conversions services

Dan Pierce
2017-03-13
"""
import sys
import rospy
from wgs_conversions.srv import WgsConversion

def wgs_client(x,y,z):
	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	"" Convert from XYZ to LLA
	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	rospy.wait_for_service('xyz2lla')

	xyz2lla = rospy.ServiceProxy('xyz2lla', WgsConversion)
	rsp = xyz2lla(xyz=(x,y,z))
	(lat,lon,alt)=rsp.lla

	print "\nSuccessfully converted from ecef to lla"
	print "\tLat: ",lat,"\tLong: ",lon,"\tAlt: ",alt

	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	"" Convert from LLA to XYZ
	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	rospy.wait_for_service('lla2xyz')

	lla2xyz = rospy.ServiceProxy('lla2xyz', WgsConversion)
	rsp = lla2xyz(lla=(lat,lon,alt))
	(x,y,z)=rsp.xyz

	print "\nSuccessfully converted from lla back to ecef"
	print "\tX: ",x,"\tY: ",y,"\tZ: ",z

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
	
	print "\nSuccessfully converted from lla to enu"
	print "\tE: ",e,"\tN: ",n,"\tU: ",u

	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	"" Convert from ENU to LLA
	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	rospy.wait_for_service('enu2lla')

	enu2lla = rospy.ServiceProxy('enu2lla', WgsConversion)
	rsp = enu2lla(enu=(e,n,u),ref_lla=(ref_lat,ref_lon,ref_alt))
	(lat,lon,alt)=rsp.lla
	
	print "\nSuccessfully converted from enu back to lla"
	print "\tLat: ",lat,"\tLong: ",lon,"\tAlt: ",alt

	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	"" Convert from XYZ to ENU
	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	rospy.wait_for_service('xyz2enu')

	xyz2enu = rospy.ServiceProxy('xyz2enu', WgsConversion)
	rsp = xyz2enu(xyz=(x,y,z),ref_lla=(ref_lat,ref_lon,ref_alt))
	(e,n,u)=rsp.enu
	
	print "\nSuccessfully converted from xyz to enu"
	print "\tE: ",e,"\tN: ",n,"\tU: ",u

	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	"" Convert from ENU to XYZ
	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	rospy.wait_for_service('enu2xyz')

	enu2xyz = rospy.ServiceProxy('enu2xyz', WgsConversion)
	rsp = enu2xyz(enu=(e,n,u),ref_lla=(ref_lat,ref_lon,ref_alt))
	(x,y,z)=rsp.xyz
	
	print "\nSuccessfully converted from enu back to xyz"
	print "\tX: ",x,"\tY: ",y,"\tZ: ",z

	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	"" Convert velocities from XYZ to ENU
	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	vx = 23.3883
	vy = 2.1984
	vz = 0.4462

	rospy.wait_for_service('xyz2enu_vel')

	xyz2enu_vel = rospy.ServiceProxy('xyz2enu_vel', WgsConversion)
	rsp = xyz2enu_vel(xyz=(vx,vy,vz),ref_lla=(ref_lat,ref_lon,ref_alt))
	(ve,vn,vu)=rsp.enu
	
	print "\nSuccessfully converted velocities from xyz to enu"
	print "\tvE: ",ve,"\tvN: ",vn,"\tvU: ",vu

	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	"" Convert velocities from ENU to XYZ
	"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
	rospy.wait_for_service('enu2xyz_vel')

	enu2xyz_vel = rospy.ServiceProxy('enu2xyz_vel', WgsConversion)
	rsp = enu2xyz_vel(enu=(ve,vn,vu),ref_lla=(ref_lat,ref_lon,ref_alt))
	(vx,vy,vz)=rsp.xyz
	
	print "\nSuccessfully converted velocities from enu back to xyz"
	print "\tvX: ",vx,"\tvY: ",vy,"\tvZ: ",vz

if __name__ == "__main__":
	if len(sys.argv) == 4:
		ECEFx = float(sys.argv[1])
		ECEFy = float(sys.argv[2])
		ECEFz = float(sys.argv[3])
	else:
		print "\nUsage: wgs_conversions_client ECEF_X ECEF_Y ECEF_Z. Using default values:"
		ECEFx = 422143.588316416
		ECEFy = -5361864.950247487
		ECEFz = 3417050.753609172
		print "\tX: ",ECEFx,"\tY: ",ECEFy,"\tZ: ",ECEFz

	wgs_client(ECEFx,ECEFy,ECEFz)