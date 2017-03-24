/*
 * Header file for wgs_conversions.cpp
 *
 * Dan Pierce
 * 2017-03-13
 */
#include <iostream>
#include <math.h>

typedef double array_type[3];
typedef double matrix_type[3][3];

/*! Primary class for wgs_conversions */
class WgsConversions{

  public:
    WgsConversions();
    ~WgsConversions();

    /*! Convert to/from ENU/LLA (requires reference LLA) */
    bool enu2lla(double enu[3], double ref_lla[3], array_type& lla);
    bool lla2enu(double lla[3], double ref_lla[3], array_type& enu);

    /*! Convert to/from ECEF/LLA */
    bool xyz2lla(double xyz[3], array_type& lla);
    bool lla2xyz(double lla[3], array_type& xyz);

    /*! Convert to/from ENU/ECEF (requires reference LLA) */
    bool enu2xyz(double enu[3], double ref_lla[3], array_type& xyz);
    bool xyz2enu(double xyz[3], double ref_lla[3], array_type& enu);

    /*! Convert velocities (or delta positions) to/from ENU/ECEF (requires reference LLA) */
    void enu2xyz_vel(double enu_vel[3], double ref_lla[3], array_type& xyz_vel);
    void xyz2enu_vel(double xyz_vel[3], double ref_lla[3], array_type& enu_vel);

  private:

    /*! Rotation matrix about a given axis */
    void rot(double R[3][3], double angle, int axis);
    
    /*! Rotation matrix from ECEF to ENU frame */
    void rot3d(double R[3][3], double reflat, double reflon);
    
    /*! Multiply 3x3 matrix times another 3x3 matrix C=AB */
    void matrixMultiply(double C[3][3], double A[3][3], double B[3][3]);
    
    /*! Multiply 3x3 matrix times a 3x1 vector c=Ab */
    void matrixMultiply(double c[3], double A[3][3], double b[3]);
    
    /*! Transpose a 3x3 matrix At = A' */
    void transposeMatrix(double At[3][3], double A[3][3]);
    
};
