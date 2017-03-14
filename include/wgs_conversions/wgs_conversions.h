/*
 * Header file for wgs_conversions.cpp
 *
 * Dan Pierce
 * 2017-03-13
 */

#include <iostream>
#include <math.h>

typedef double array_type[3];

/*! /brief Primary class for wgs_conversions
*
*/
class WgsConversions
{
  public:

    WgsConversions();
    ~WgsConversions();

    bool enu2lla(double enu[3], double ref_lla[3], array_type& lla);
    bool lla2enu(double lla[3], double ref_lla[3], array_type& enu);

    bool xyz2lla(double xyz[3], array_type& lla);
    bool lla2xyz(double lla[3], array_type& xyz);

    bool enu2xyz(double enu[3], double ref_lla[3], array_type& xyz);
    bool xyz2enu(double xyz[3], double ref_lla[3], array_type& enu);

  private:

    void rot(double R[3][3], double angle, int axis);
    
};
