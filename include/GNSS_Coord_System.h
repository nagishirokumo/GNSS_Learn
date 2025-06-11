#ifndef _GNSS_COORD_SYSTEM_H
#define _GNSS_COORD_SYSTEM_H
#include "Matrix_Calculate.h"

#define PI 3.1415926535898
#define WGS_84_A 6378137
#define WGS_84_B 6356752.31424517
#define WGS_84_E 0.081819190842622
#define WGS_84_E2 0.0066943799013

class Rinex_Navigation_Message_Data;

class Cartesian_XYZ_Coord
{
    friend void Write2CSV(Rinex_Navigation_Message_Data &data);
public:
    double X,Y,Z;
public:
    Cartesian_XYZ_Coord(double X_,double Y_,double Z_):X(X_),Y(Y_),Z(Z_){}
    Cartesian_XYZ_Coord(){}
//    double calcZ();
//    int XYZ2BLH();
//    int BLH2XYZ();
//    void coutBLH();
//    void coutXYZ();
};

class BLH_Coord
{
public:
    double B,L,H;
    BLH_Coord(){}
    BLH_Coord(double B_,double L_,double H_)
    :B(B_),L(L_),H(H_){}
};


class ENU
{
public:
    double E,N,U;
    ENU(){};
    ENU(double E_,double N_,double U_):E(E_),N(N_),U(U_){};
};

ENU XYZ2ENU(Cartesian_XYZ_Coord XYZ,Cartesian_XYZ_Coord Refer_XYZ);

BLH_Coord XYZ2BLH(Cartesian_XYZ_Coord XYZ);

Cartesian_XYZ_Coord BLH2XYZ(BLH_Coord BLH);

#endif