/*
 * An example client for using the wgs_conversions services 
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

  std::cout.precision(15);

  if (argc != 4){
    std::cout << "\nUsage: wgs_conversions_client ECEF_X ECEF_Y ECEF_Z. Using default values:" << std::endl;
    xyz[0]= 422143.588316416;
    xyz[1]= -5361864.950247487;
    xyz[2]= 3417050.753609172;
    std::cout << "\tX: " << xyz[0] << "\tY: " << xyz[1] << "\tZ: " << xyz[2] << std::endl;
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
    std::cout << "\nSuccessfully converted from ecef to lla" << std::endl;
    std::cout << "\tLat: " << lla[0] << "\tLong: " << lla[1] << "\tAlt: " << lla[2] << std::endl;
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
    std::cout << "\nSuccessfully converted from lla back to ecef" << std::endl;
    std::cout << "\tX: " << xyz[0] << "\tY: " << xyz[1] << "\tZ: " << xyz[2] << std::endl;
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
    std::cout << "\nSuccessfully converted from lla to enu" << std::endl;
    std::cout << "\tE: " << enu[0] << "\tN: " << enu[1] << "\tU: " << enu[2] << std::endl;
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
    std::cout << "\nSuccessfully converted from enu back to lla" << std::endl;
    std::cout << "\tLat: " << lla[0] << "\tLong: " << lla[1] << "\tAlt: " << lla[2] << std::endl;
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
    std::cout << "\nSuccessfully converted from ecef to enu" << std::endl;
    std::cout << "\tE: " << enu[0] << "\tN: " << enu[1] << "\tU: " << enu[2] << std::endl;
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
    std::cout << "\nSuccessfully converted from enu back to ecef" << std::endl;
    std::cout << "\tX: " << xyz[0] << "\tY: " << xyz[1] << "\tZ: " << xyz[2] << std::endl;
  }else{
    ROS_ERROR("Failed to call service enu2xyz");
    return 1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert velocities from XYZ to ENU
  ////////////////////////////////////////////////////////////////////////////////
  client = nh.serviceClient<wgs_conversions::WgsConversion>("xyz2enu_vel");

  double xyz_vel[3],enu_vel[3];

  xyz_vel[0] = 23.3883;
  xyz_vel[1] = 2.1984;
  xyz_vel[2] = 0.4462;

  srv.request.xyz[0] = xyz_vel[0];
  srv.request.xyz[1] = xyz_vel[1];
  srv.request.xyz[2] = xyz_vel[2];

  srv.request.ref_lla[0] = ref_lla[0];
  srv.request.ref_lla[1] = ref_lla[1];
  srv.request.ref_lla[2] = ref_lla[2];

  return_status = client.call(srv);

  if (return_status){
    enu_vel[0] = srv.response.enu[0];
    enu_vel[1] = srv.response.enu[1];
    enu_vel[2] = srv.response.enu[2];
    std::cout << "\nSuccessfully converted velocities from ecef to enu" << std::endl;
    std::cout << "\tvE: " << enu_vel[0] << "\tvN: " << enu_vel[1] << "\tvU: " << enu_vel[2] << std::endl;
  }else{
    ROS_ERROR("Failed to call service xyz2enu_vel");
    return 1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert velocities from ENU to XYZ
  ////////////////////////////////////////////////////////////////////////////////
  client = nh.serviceClient<wgs_conversions::WgsConversion>("enu2xyz_vel");

  srv.request.enu[0] = enu_vel[0];
  srv.request.enu[1] = enu_vel[1];
  srv.request.enu[2] = enu_vel[2];

  srv.request.ref_lla[0] = ref_lla[0];
  srv.request.ref_lla[1] = ref_lla[1];
  srv.request.ref_lla[2] = ref_lla[2];

  return_status = client.call(srv);

  if (return_status){
    xyz_vel[0] = srv.response.xyz[0];
    xyz_vel[1] = srv.response.xyz[1];
    xyz_vel[2] = srv.response.xyz[2];
    std::cout << "\nSuccessfully converted velocities from enu back to ecef" << std::endl;
    std::cout << "\tvX: " << xyz_vel[0] << "\tvY: " << xyz_vel[1] << "\tvZ: " << xyz_vel[2] << std::endl;
  }else{
    ROS_ERROR("Failed to call service enu2xyz_vel");
    return 1;
  }


  return 0;
}
