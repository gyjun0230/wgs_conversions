/*
 * A node
 *
 * Dan Pierce
 * 2017-03-13
 */
#include <ros/ros.h>

#include "wgs_conversions/WgsConversion.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wgs_conversions_client");

  int return_status;
  double xyz[3],lla[3],ref_lla[3],enu[3];

  if (argc != 4){
    ROS_INFO("Usage: wgs_conversions_client ECEF_X ECEF_Y ECEF_Z. Using default values:");
    xyz[0]= 422143.588316416;
    xyz[1]= -5361864.950247487;
    xyz[2]= 3417050.753609172;
    ROS_INFO("\tX: %f\tY: %f\tZ: %f\n", xyz[0], xyz[1], xyz[2]);
  }else{
    xyz[0]= atof(argv[1]);
    xyz[1]= atof(argv[2]);
    xyz[2]= atof(argv[3]);
  }

  ros::NodeHandle nh;
  ros::ServiceClient client;
  wgs_conversions::WgsConversion srv;

  ////////////////////////////////////////////////////////////////////////////////
  // Convert from XYZ to LLA
  ////////////////////////////////////////////////////////////////////////////////
  client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2lla");
  
  srv.request.xyz[0] = xyz[0];
  srv.request.xyz[1] = xyz[1];
  srv.request.xyz[2] = xyz[2];

  return_status = client.call(srv);

  if (return_status){
    
    lla[0] = srv.response.lla[0];
    lla[1] = srv.response.lla[1];
    lla[2] = srv.response.lla[2];

    ROS_INFO("Successfully converted from ecef to lla");
    ROS_INFO("\tLat: %f\tLong: %f\tAlt: %f\n", lla[0], lla[1], lla[2]);

  }else{
    ROS_ERROR("Failed to call service xyz2lla");
    return 1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert from LLA to XYZ
  ////////////////////////////////////////////////////////////////////////////////
  client = nh.serviceClient<wgs_conversions::WgsConversion>("lla2xyz");

  srv.request.lla[0] = lla[0];
  srv.request.lla[1] = lla[1];
  srv.request.lla[2] = lla[2];

  return_status = client.call(srv);

  if (return_status){

    xyz[0] = srv.response.xyz[0];
    xyz[1] = srv.response.xyz[1];
    xyz[2] = srv.response.xyz[2];

    ROS_INFO("Successfully converted lla back to ecef");
    ROS_INFO("\tX: %f\tY: %f\tZ: %f\n", xyz[0], xyz[1], xyz[2]);

  }else{
    ROS_ERROR("Failed to call service lla2xyz");
    return 1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert from LLA to ENU
  ////////////////////////////////////////////////////////////////////////////////
  client = nh.serviceClient<wgs_conversions::WgsConversion>("lla2enu");

  ref_lla[0] = lla[0] + 0.01;
  ref_lla[1] = lla[1] + 0.01;
  ref_lla[2] = lla[2] + 10.0;
  
  srv.request.lla[0] = lla[0];
  srv.request.lla[1] = lla[1];
  srv.request.lla[2] = lla[2];

  srv.request.ref_lla[0] = ref_lla[0];
  srv.request.ref_lla[1] = ref_lla[1];
  srv.request.ref_lla[2] = ref_lla[2];

  return_status = client.call(srv);

  if (return_status){

    enu[0] = srv.response.enu[0];
    enu[1] = srv.response.enu[1];
    enu[2] = srv.response.enu[2];

    ROS_INFO("Successfully converted lla to enu");
    ROS_INFO("\tE: %f\tN: %f\tU: %f\n", enu[0], enu[1], enu[2]);

  }else{
    ROS_ERROR("Failed to call service lla2enu");
    return 1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert from ENU to LLA
  ////////////////////////////////////////////////////////////////////////////////
  client = nh.serviceClient<wgs_conversions::WgsConversion>("enu2lla");

  srv.request.enu[0] = enu[0];
  srv.request.enu[1] = enu[1];
  srv.request.enu[2] = enu[2];

  srv.request.ref_lla[0] = ref_lla[0];
  srv.request.ref_lla[1] = ref_lla[1];
  srv.request.ref_lla[2] = ref_lla[2];

  return_status = client.call(srv);

  if (return_status){

    lla[0] = srv.response.lla[0];
    lla[1] = srv.response.lla[1];
    lla[2] = srv.response.lla[2];

    ROS_INFO("Successfully converted enu back to lla");
    ROS_INFO("\tLat: %f\tLong: %f\tAlt: %f\n", lla[0], lla[1], lla[2]);

  }else{
    ROS_ERROR("Failed to call service enu2lla");
    return 1;
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // Convert from XYZ to ENU
  ////////////////////////////////////////////////////////////////////////////////
  client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2enu");

  srv.request.xyz[0] = xyz[0];
  srv.request.xyz[1] = xyz[1];
  srv.request.xyz[2] = xyz[2];

  srv.request.ref_lla[0] = ref_lla[0];
  srv.request.ref_lla[1] = ref_lla[1];
  srv.request.ref_lla[2] = ref_lla[2];

  return_status = client.call(srv);

  if (return_status){

    enu[0] = srv.response.enu[0];
    enu[1] = srv.response.enu[1];
    enu[2] = srv.response.enu[2];

    ROS_INFO("Successfully converted xyz to enu");
    ROS_INFO("\tE: %f\tN: %f\tU: %f\n", enu[0], enu[1], enu[2]);

  }else{
    ROS_ERROR("Failed to call service xyz2enu");
    return 1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert from ENU to XYZ
  ////////////////////////////////////////////////////////////////////////////////
  client = nh.serviceClient<wgs_conversions::WgsConversion>("enu2xyz");

  srv.request.enu[0] = enu[0];
  srv.request.enu[1] = enu[1];
  srv.request.enu[2] = enu[2];

  srv.request.ref_lla[0] = ref_lla[0];
  srv.request.ref_lla[1] = ref_lla[1];
  srv.request.ref_lla[2] = ref_lla[2];

  return_status = client.call(srv);

  if (return_status){

    xyz[0] = srv.response.xyz[0];
    xyz[1] = srv.response.xyz[1];
    xyz[2] = srv.response.xyz[2];

    ROS_INFO("Successfully converted enu back to xyz");
    ROS_INFO("\tLat: %f\tLong: %f\tAlt: %f\n", xyz[0], xyz[1], xyz[2]);

  }else{
    ROS_ERROR("Failed to call service enu2xyz");
    return 1;
  }


  return 0;
}
