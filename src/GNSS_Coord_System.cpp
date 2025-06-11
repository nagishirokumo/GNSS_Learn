#include "GNSS_Coord_System.h"
#include <cmath>

BLH_Coord XYZ2BLH(Cartesian_XYZ_Coord XYZ) {
    const double eps = 1e-12;
    double X = XYZ.X, Y = XYZ.Y, Z = XYZ.Z;

    double L = atan2(Y, X);
    double R = sqrt(X*X + Y*Y);
    double B = atan2(Z, R * (1 - WGS_84_E2));  // 更好的初值
    double N = 0, H = 0, B_prev;

    do {
        B_prev = B;
        N = WGS_84_A / sqrt(1 - WGS_84_E2 * sin(B) * sin(B));
        H = R / cos(B) - N;
        B = atan2(Z, R * (1 - WGS_84_E2 * N / (N + H)));
    } while (fabs(B - B_prev) > eps);

    return BLH_Coord(B, L, H);
}

Cartesian_XYZ_Coord BLH2XYZ(BLH_Coord BLH)
{
    double B,L,H,X,Y,Z;
    B=BLH.B;
    H=BLH.H;
    L=BLH.L;
    double N=WGS_84_A/sqrt(1-WGS_84_E2*sin(B)*sin(B));
    X=(N+H)*cos(B)*cos(L);
    Y=(N+H)*cos(B)*sin(L);
    Z=(N*(1-WGS_84_E2)+H)*sin(B);

    Cartesian_XYZ_Coord res(X,Y,Z);
    return res;
}

ENU XYZ2ENU(Cartesian_XYZ_Coord XYZ,Cartesian_XYZ_Coord Refer_XYZ)
{
    BLH_Coord Refer_BLH=XYZ2BLH(Refer_XYZ);
    double Tran_[9]={-1.0*std::sin(Refer_BLH.L),std::cos(Refer_BLH.L),0,
                        -1.0*std::sin(Refer_BLH.B)*std::cos(Refer_BLH.L),-1.0*std::sin(Refer_BLH.B)*std::sin(Refer_BLH.L),std::cos(Refer_BLH.B),
                        std::cos(Refer_BLH.B)*std::cos(Refer_BLH.L),std::cos(Refer_BLH.B)*std::sin(Refer_BLH.L),std::sin(Refer_BLH.B)};
    Matrix Tran(3,3,Tran_);

    double dXYZ_[3]={XYZ.X-Refer_XYZ.X,XYZ.Y-Refer_XYZ.Y,XYZ.Z-Refer_XYZ.Z};
    Matrix dXYZ(3,1,dXYZ_);

    Matrix vENU=Tran*dXYZ;
    ENU Res(vENU.My_Matrix(0,0),vENU.My_Matrix(1,0),vENU.My_Matrix(2,0));
    return Res;
}






