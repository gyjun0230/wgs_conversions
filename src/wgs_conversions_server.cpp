/*
 * A ROS node to provide services for performing WGS conversions. This code acts as a wrapper 
 * for the WgsConversions class.
 *
 * Dan Pierce
 * 2017-03-13
 */
#include <ros/ros.h>
#include "wgs_conversions/wgs_conversions.h"
#include "wgs_conversions/WgsConversion.h"

/*! Primary class for WGS conversions server */
class WgsConversionsServer{

	public:
		WgsConversionsServer();
		~WgsConversionsServer();

	private:

		bool xyz2lla(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp);
		bool lla2xyz(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp);
		
		bool lla2enu(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp);
		bool enu2lla(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp);

		bool xyz2enu(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp);
		bool enu2xyz(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp);
		
		bool xyz2enu_vel(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp);
		bool enu2xyz_vel(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp);
		
		WgsConversions wgs; /*!< WgsConversions instance */

    	ros::NodeHandle nh;

		ros::ServiceServer xyz2lla_service;
		ros::ServiceServer lla2xyz_service;

		ros::ServiceServer lla2enu_service;
		ros::ServiceServer enu2lla_service;

		ros::ServiceServer xyz2enu_service;
		ros::ServiceServer enu2xyz_service;

		ros::ServiceServer xyz2enu_vel_service;
		ros::ServiceServer enu2xyz_vel_service;
    
};

WgsConversionsServer::WgsConversionsServer(){
    
	xyz2lla_service = nh.advertiseService("xyz2lla", &WgsConversionsServer::xyz2lla, this);
	lla2xyz_service = nh.advertiseService("lla2xyz", &WgsConversionsServer::lla2xyz, this);
	
	lla2enu_service = nh.advertiseService("lla2enu", &WgsConversionsServer::lla2enu, this);
	enu2lla_service = nh.advertiseService("enu2lla", &WgsConversionsServer::enu2lla, this);

	xyz2enu_service = nh.advertiseService("xyz2enu", &WgsConversionsServer::xyz2enu, this);
	enu2xyz_service = nh.advertiseService("enu2xyz", &WgsConversionsServer::enu2xyz, this);
	
	xyz2enu_vel_service = nh.advertiseService("xyz2enu_vel", &WgsConversionsServer::xyz2enu_vel, this);
	enu2xyz_vel_service = nh.advertiseService("enu2xyz_vel", &WgsConversionsServer::enu2xyz_vel, this);
	
	ROS_INFO("Ready for WGS conversions");
	
}

WgsConversionsServer::~WgsConversionsServer(){

}

bool WgsConversionsServer::xyz2lla(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp){
	
	double xyz[3],lla[3];

	for(int i=0;i<3;i++)
		xyz[i]=req.xyz[i];

	if(!wgs.xyz2lla(xyz,lla))
		return false;

	for(int i=0;i<3;i++)
		rsp.lla[i]=lla[i];

  	return true;
}

bool WgsConversionsServer::lla2xyz(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp){
	
	double lla[3],xyz[3];

	for(int i=0;i<3;i++)
		lla[i]=req.lla[i];

	if(!wgs.lla2xyz(lla,xyz))
		return false;

	for(int i=0;i<3;i++)
		rsp.xyz[i]=xyz[i];

  	return true;
}

bool WgsConversionsServer::lla2enu(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp){
	
	double lla[3],ref_lla[3],enu[3];

	for(int i=0;i<3;i++){
		lla[i]=req.lla[i];
		ref_lla[i]=req.ref_lla[i];
	}

	if(!wgs.lla2enu(lla,ref_lla,enu))
		return false;

	for(int i=0;i<3;i++)
		rsp.enu[i]=enu[i];

  	return true;
}

bool WgsConversionsServer::enu2lla(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp){
	
	double enu[3],ref_lla[3],lla[3];

	for(int i=0;i<3;i++){
		enu[i]=req.enu[i];
		ref_lla[i]=req.ref_lla[i];
	}

	if(!wgs.enu2lla(enu,ref_lla,lla))
		return false;

	for(int i=0;i<3;i++)
		rsp.lla[i]=lla[i];

  	return true;
}

bool WgsConversionsServer::xyz2enu(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp){
	
	double xyz[3],ref_lla[3],enu[3];

	for(int i=0;i<3;i++){
		xyz[i]=req.xyz[i];
		ref_lla[i]=req.ref_lla[i];
	}

	if(!wgs.xyz2enu(xyz,ref_lla,enu))
		return false;

	for(int i=0;i<3;i++)
		rsp.enu[i]=enu[i];

  	return true;
}

bool WgsConversionsServer::enu2xyz(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp){
	
	double enu[3],ref_lla[3],xyz[3];

	for(int i=0;i<3;i++){
		enu[i]=req.enu[i];
		ref_lla[i]=req.ref_lla[i];
	}

	if(!wgs.enu2xyz(enu,ref_lla,xyz))
		return false;

	for(int i=0;i<3;i++)
		rsp.xyz[i]=xyz[i];

  	return true;
}

bool WgsConversionsServer::xyz2enu_vel(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp){
	
	double xyz_vel[3],ref_lla[3],enu_vel[3];

	for(int i=0;i<3;i++){
		xyz_vel[i]=req.xyz[i];
		ref_lla[i]=req.ref_lla[i];
	}

	wgs.xyz2enu_vel(xyz_vel,ref_lla,enu_vel);

	for(int i=0;i<3;i++)
		rsp.enu[i]=enu_vel[i];

  	return true;
}

bool WgsConversionsServer::enu2xyz_vel(wgs_conversions::WgsConversion::Request &req,wgs_conversions::WgsConversion::Response &rsp){
	
	double enu_vel[3],ref_lla[3],xyz_vel[3];

	for(int i=0;i<3;i++){
		enu_vel[i]=req.enu[i];
		ref_lla[i]=req.ref_lla[i];
	}

	wgs.enu2xyz_vel(enu_vel,ref_lla,xyz_vel);
	
	for(int i=0;i<3;i++)
		rsp.xyz[i]=xyz_vel[i];

  	return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wgs_conversions_server");
	
	WgsConversionsServer server;

	ros::spin();
	
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
