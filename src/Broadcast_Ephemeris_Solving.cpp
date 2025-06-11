//
// Created by asus on 2025/6/5.
//
#include "DATA_Novatel.h"
//GPS卫星位置解算
void GPS_PVT(const Novatel_Message_GPSEPHEM &GPSEPHEM, Novatel_Data &Novatel_Data)
{
    int deltat=-3600;//GPS广播星历间隔2小时
    int step=300;//解算步长
    while(deltat<3600)
    {
        double n0 = 0;//平均运动速率(未修正)
        double n = 0;//平均运动速率(修正)
        n0 = std::sqrt(GPS_GM) / GPSEPHEM.sqrtA / GPSEPHEM.sqrtA / GPSEPHEM.sqrtA;
        n = n0 + GPSEPHEM.Delta_N;

        double M_cur = GPSEPHEM.M0 + n * deltat;//计算平近点角

        double E_cur, E_tmp;//偏近点角
//        E_cur = 2;
//        E_tmp = 1;
//        while (fabs(E_cur - E_tmp) > 1e-9) {
//            E_tmp = E_cur;
//            E_cur = sin(E_tmp) * GPSEPHEM.e + M_cur;
//        }
        E_cur = M_cur;
        do {
            E_tmp = E_cur;
            E_cur = E_tmp - (E_tmp - GPSEPHEM.e * sin(E_tmp) - M_cur) / (1 - GPSEPHEM.e * cos(E_tmp));
        } while (fabs(E_cur - E_tmp) > 1e-12);

        //钟差改正
        double t_corr=GPSEPHEM.sa0+GPSEPHEM.sa1*deltat+GPSEPHEM.sa2*deltat*deltat
                      -GPSEPHEM.Tgd+DELTAT_CORR_F*GPSEPHEM.e*GPSEPHEM.sqrtA*sin(E_cur);
//        double deltat_corr=deltat-t_corr;
        double deltat_corr=deltat;

        //计算真近点角
        double f_cur = atan2(sqrt(1 - GPSEPHEM.e * GPSEPHEM.e) * sin(E_cur), cos(E_cur) - GPSEPHEM.e);
        //计算升交距角（未经改正的）
        double u_cur_nocorr = GPSEPHEM.omega + f_cur;
        //计算卫星向径（未经改正的）
        double r_cur_nocorr = GPSEPHEM.sqrtA * GPSEPHEM.sqrtA * (1 - cos(E_cur) * GPSEPHEM.e);

        //计算摄动改正项
        double u_corr = GPSEPHEM.Cus * sin(2 * u_cur_nocorr) + GPSEPHEM.Cuc * cos(2 * u_cur_nocorr);
        double r_corr = GPSEPHEM.Crs * sin(2 * u_cur_nocorr) + GPSEPHEM.Crc * cos(2 * u_cur_nocorr);
        double i_corr = GPSEPHEM.Cis * sin(2 * u_cur_nocorr) + GPSEPHEM.Cic * cos(2 * u_cur_nocorr);

        //进行摄动改正
        double u_cur = u_cur_nocorr + u_corr;
        double r_cur = r_cur_nocorr + r_corr;
        double i_cur = GPSEPHEM.I0 + GPSEPHEM.I_Dot * deltat_corr + i_corr;

        //计算卫星在轨道平面坐标系（xoy）中的位置
        double x_cur = r_cur * cos(u_cur);
        double y_cur = r_cur * sin(u_cur);

        //计算观测瞬间升交点经度（大地经度=赤经减去GAST）
        //t时刻升交点赤经
        //double OMEGA_cur=GPSEPHEM.OMEGA0+GPSEPHEM.OMEGADOT*deltat;

        //t时刻大地经度
        double Longitude_cur = GPSEPHEM.OMEGA0 + GPSEPHEM.OMEGADOT * deltat_corr - (deltat_corr + GPSEPHEM.Toe) * GPS_OMEGAE;

        //卫星在ECEF瞬时地心地固坐标系下的坐标
        double X = x_cur * cos(Longitude_cur) - y_cur * cos(i_cur) * sin(Longitude_cur);
        double Y = x_cur * sin(Longitude_cur) + y_cur * cos(i_cur) * cos(Longitude_cur);
        double Z = y_cur * sin(i_cur);

        //极移修正
//        double xp = 0.0634 * PI/648000.0;
//        double yp = 0.3316 * PI/648000.0;
//        double X_Polar_corr = X + xp * Z;
//        double Y_Polar_corr = Y - yp * Z;
//        double Z_Polar_corr = - xp * X + yp * Y + Z;

        // 速度计算
        double E_cur_Dot=n/(1-GPSEPHEM.e*cos(E_cur));
        double Phi_cur_Dot= sqrt((1+GPSEPHEM.e)/(1-GPSEPHEM.e))*( cos(f_cur/2)/ cos(E_cur/2) )
                            *( cos(f_cur/2)/ cos(E_cur/2) )*E_cur_Dot;
        double u_k_Dot=2*(GPSEPHEM.Cus*cos(2*u_cur_nocorr)-GPSEPHEM.Cuc*sin(2*u_cur_nocorr))*Phi_cur_Dot+Phi_cur_Dot;
        double r_k_Dot=GPSEPHEM.A*GPSEPHEM.e* sin(E_cur)* sin(E_cur_Dot)+
                       2*(GPSEPHEM.Crs*cos(2*u_cur_nocorr)-GPSEPHEM.Crc*sin(2*u_cur_nocorr))*Phi_cur_Dot;
        double I_k_Dot=GPSEPHEM.I_Dot+2*(GPSEPHEM.Cis*cos(2*u_cur_nocorr)-GPSEPHEM.Cic*sin(2*u_cur_nocorr))*Phi_cur_Dot;
        double OMEGA_k_Dot=GPSEPHEM.OMEGADOT-GPS_OMEGAE;
        //轨道平面坐标导数
        double x_k_Dot=r_k_Dot*cos(u_cur)-r_cur*u_k_Dot* sin(u_cur);
        double y_k_Dot=r_k_Dot*sin(u_cur)+r_cur*u_k_Dot* cos(u_cur);

        //卫星速度
        double X_Dot= cos(Longitude_cur)*x_k_Dot + (-1* sin(Longitude_cur))* cos(i_cur)*y_k_Dot
                      + -1 *(x_cur* sin(Longitude_cur)+y_cur* cos(Longitude_cur)* cos(i_cur))*OMEGA_k_Dot
                      +y_cur* sin(Longitude_cur)* sin(i_cur)*I_k_Dot;
        double Y_Dot=sin(Longitude_cur)*x_k_Dot + (cos(Longitude_cur))* cos(i_cur)*y_k_Dot
                     + (x_cur* cos(Longitude_cur)-y_cur* sin(Longitude_cur)* cos(i_cur))*OMEGA_k_Dot
                     +y_cur* cos(Longitude_cur)* sin(i_cur)*I_k_Dot;

        double Z_Dot= sin(i_cur)*y_k_Dot+y_cur* cos(i_cur)*I_k_Dot;

        Cartesian_XYZ_Coord Pos(X,Y,Z);
        GPST GPStime(GPSEPHEM.Week,GPSEPHEM.Toe+deltat);
        GPS_Position Tmp;
        Tmp.GPS_Time=GPStime;
        Tmp.Position=Pos;
        Tmp.v_X=X_Dot;
        Tmp.v_Y=Y_Dot;
        Tmp.v_Z=Z_Dot;
        Tmp.Clkdelta=t_corr;
        Tmp.Clkdelta_Dot=GPSEPHEM.sa1+2*GPSEPHEM.sa2*deltat+DELTAT_CORR_F*GPSEPHEM.e*GPSEPHEM.sqrtA* cos(E_cur)*E_cur_Dot;
        Novatel_Data.GPS_Position[GPSEPHEM.PRN].push_back(Tmp);

//        printf("PRN:%d\n",GPSEPHEM.PRN);
//        printf("GPST: Week-%d\tSecond-%d\n",GPStime.GPS_Week,GPStime.GPS_Second);
//        printf("Position: X-%lf\tY-%lf\tZ-%lf\n",X,Y,Z);
//        printf("Velocity: X-%lf\tY-%lf\tZ-%lf\n",X_Dot,Y_Dot,Z_Dot);
//        printf("Clkdelta: %.9lf\t Clkdelta_Dot: %.15lf\n",Tmp.Clkdelta,Tmp.Clkdelta_Dot);

//        double refTime=0;//2019 12 01 00 00 00 周内秒为0
//        double t_cur = deltat_corr + GPSEPHEM.Toe;
//        double dt    = t_cur - refTime;
//        double theta = omega_e * dt;
//
//        // 把瞬时 ECEF 逆旋转到参考时刻 ECEF
//        double cos_t = std::cos(theta);
//        double sin_t = std::sin(theta);
//        double X_ref =  cos_t * X_Polar_corr - sin_t * Y_Polar_corr;
//        double Y_ref =  sin_t * X_Polar_corr + cos_t * Y_Polar_corr;
//        double Z_ref =  Z_Polar_corr;
        deltat+=step;
    }
}
GPS_Position GPS_PVT(const Novatel_Message_GPSEPHEM &GPSEPHEM, Novatel_Data &Novatel_Data, GPST Positioning_Time, double Psr)
{
    double deltat=Positioning_Time-GPST(GPSEPHEM.Week,GPSEPHEM.Toe);
    double deltat_corr= deltat - Psr / C_LIGHT;
    double n0 = 0;//平均运动速率(未修正)
    double n = 0;//平均运动速率(修正)
    n0 = std::sqrt(GPS_GM) / GPSEPHEM.sqrtA / GPSEPHEM.sqrtA / GPSEPHEM.sqrtA;
    n = n0 + GPSEPHEM.Delta_N;

    double M_cur = GPSEPHEM.M0 + n * deltat_corr;//计算平近点角

    double E_cur, E_tmp;//偏近点角
    E_cur = M_cur;
    do {
        E_tmp = E_cur;
        E_cur = E_tmp - (E_tmp - GPSEPHEM.e * std::sin(E_tmp) - M_cur) / (1 - GPSEPHEM.e * std::cos(E_tmp));
    } while (std::fabs(E_cur - E_tmp) > 1e-12);

    //钟差改正
    double t_corr=GPSEPHEM.sa0+GPSEPHEM.sa1*deltat+GPSEPHEM.sa2*deltat*deltat
                  -GPSEPHEM.Tgd+DELTAT_CORR_F*GPSEPHEM.e*GPSEPHEM.sqrtA*sin(E_cur);
    deltat_corr=deltat_corr-t_corr;//经过这一步修正后，计算的是卫星信号发射时刻的卫星坐标

    //计算真近点角
    double v_cur = atan2(std::sqrt(1 - GPSEPHEM.e * GPSEPHEM.e) * std::sin(E_cur), std::cos(E_cur) - GPSEPHEM.e);

    //计算升交距角（未经改正的）
    double u_cur_nocorr = GPSEPHEM.omega + v_cur;
    //计算卫星向径（未经改正的）
    double r_cur_nocorr = GPSEPHEM.sqrtA * GPSEPHEM.sqrtA * (1 - std::cos(E_cur) * GPSEPHEM.e);

    //计算摄动改正项
    double u_corr = GPSEPHEM.Cus * std::sin(2 * u_cur_nocorr) + GPSEPHEM.Cuc * std::cos(2 * u_cur_nocorr);
    double r_corr = GPSEPHEM.Crs * std::sin(2 * u_cur_nocorr) + GPSEPHEM.Crc * std::cos(2 * u_cur_nocorr);
    double i_corr = GPSEPHEM.Cis * std::sin(2 * u_cur_nocorr) + GPSEPHEM.Cic * std::cos(2 * u_cur_nocorr);

    //进行摄动改正
    double u_cur = u_cur_nocorr + u_corr;
    double r_cur = r_cur_nocorr + r_corr;
    double i_cur = GPSEPHEM.I0 + GPSEPHEM.I_Dot * deltat_corr + i_corr;

    //计算卫星在轨道平面坐标系（xoy）中的位置
    double x_cur = r_cur * std::cos(u_cur);
    double y_cur = r_cur * std::sin(u_cur);

    //计算观测瞬间升交点经度（大地经度=赤经减去GAST）
    //t时刻升交点赤经
    //double OMEGA_cur=GPSEPHEM.OMEGA0+GPSEPHEM.OMEGADOT*deltat;

    //t时刻大地经度
    double Longitude_cur = GPSEPHEM.OMEGA0 + GPSEPHEM.OMEGADOT * deltat_corr - (deltat_corr + GPSEPHEM.Toe) * GPS_OMEGAE;

    //卫星在ECEF瞬时地心地固坐标系下的坐标
    double X = x_cur * std::cos(Longitude_cur) - y_cur * std::cos(i_cur) * std::sin(Longitude_cur);
    double Y = x_cur * std::sin(Longitude_cur) + y_cur * std::cos(i_cur) * std::cos(Longitude_cur);
    double Z = y_cur * std::sin(i_cur);
    {//地球自转校正（Sagnac效应）
        double Phi_Z = GPS_OMEGAE * (Psr / C_LIGHT+t_corr);

        double Pos_[3] = {X, Y, Z};
        double Tz[9] = {std::cos(Phi_Z), std::sin(Phi_Z), 0,
                        -1.0 * std::sin(Phi_Z), std::cos(Phi_Z), 0,
                        0, 0, 1};
        Matrix T_Z(3, 3, Tz);

        Matrix Position(3, 1, Pos_);
        Matrix Position_Sagnac_corr(3, 1);
        Position_Sagnac_corr = T_Z * Position;

        X = Position_Sagnac_corr.My_Matrix(0, 0);
        Y = Position_Sagnac_corr.My_Matrix(1, 0);
        Z = Position_Sagnac_corr.My_Matrix(2, 0);
    }

    // 速度计算
    double E_cur_Dot=n/(1-GPSEPHEM.e*cos(E_cur));
    double Phi_cur_Dot= std::sqrt((1+GPSEPHEM.e)/(1-GPSEPHEM.e)) * (std::cos(v_cur / 2) / std::cos(E_cur / 2) )
                        * (std::cos(v_cur / 2) / std::cos(E_cur / 2)) * E_cur_Dot;
    double u_k_Dot=2*(GPSEPHEM.Cus*cos(2*u_cur_nocorr)-GPSEPHEM.Cuc*sin(2*u_cur_nocorr))*Phi_cur_Dot+Phi_cur_Dot;
    double r_k_Dot=GPSEPHEM.A*GPSEPHEM.e*std::sin(E_cur)*E_cur_Dot+
                   2*(GPSEPHEM.Crs*cos(2*u_cur_nocorr)-GPSEPHEM.Crc*sin(2*u_cur_nocorr))*Phi_cur_Dot;
    double I_k_Dot=GPSEPHEM.I_Dot+2*(GPSEPHEM.Cis*cos(2*u_cur_nocorr)-GPSEPHEM.Cic*sin(2*u_cur_nocorr))*Phi_cur_Dot;
    double OMEGA_k_Dot=GPSEPHEM.OMEGADOT-GPS_OMEGAE;
    //轨道平面坐标导数
    double x_k_Dot=r_k_Dot*std::cos(u_cur)-r_cur*u_k_Dot*std::sin(u_cur);
    double y_k_Dot=r_k_Dot*std::sin(u_cur)+r_cur*u_k_Dot*std::cos(u_cur);

    //卫星速度
    double R_Array[12]={
            std::cos(Longitude_cur), -1.0*std::sin(Longitude_cur)* std::cos(i_cur),
            -1.0*(x_cur*std::sin(Longitude_cur)+y_cur*std::cos(Longitude_cur)*std::cos(i_cur)),y_cur*sin(Longitude_cur)*sin(i_cur),
            sin(Longitude_cur),std::cos(Longitude_cur)*std::cos(i_cur),
            (x_cur*std::cos(Longitude_cur)-y_cur*std::sin(Longitude_cur)*std::cos(i_cur)),y_cur*std::cos(Longitude_cur)*std::sin(i_cur),
            0, std::sin(i_cur),0,y_cur*std::cos(i_cur)
    };
    double Dot_Array[4]={
            x_k_Dot,y_k_Dot,OMEGA_k_Dot,I_k_Dot
    };
    Matrix R_Matrix(3,4,R_Array);
    Matrix Dot_Vector(4,1,Dot_Array);
    Matrix Pos_Vector=R_Matrix*Dot_Vector;
    double X_Dot=Pos_Vector.My_Matrix(0,0);
    double Y_Dot=Pos_Vector.My_Matrix(1,0);;
    double Z_Dot=Pos_Vector.My_Matrix(2,0);


    Cartesian_XYZ_Coord Pos(X,Y,Z);
    GPST GPStime(GPSEPHEM.Week,GPSEPHEM.Toe+deltat_corr);
    GPS_Position Tmp;
    Tmp.GPS_Time=GPStime;
    Tmp.Position=Pos;
    Tmp.v_X=X_Dot;
    Tmp.v_Y=Y_Dot;
    Tmp.v_Z=Z_Dot;
    Tmp.Clkdelta=t_corr;
    Tmp.Clkdelta_Dot=GPSEPHEM.sa1+2*GPSEPHEM.sa2*deltat+DELTAT_CORR_F*GPSEPHEM.e*GPSEPHEM.sqrtA* cos(E_cur)*E_cur_Dot;
    Novatel_Data.GPS_Position[GPSEPHEM.PRN].push_back(Tmp);

    printf("PRN:G%d\n",GPSEPHEM.PRN);
    printf("GPST: Week-%d\tSecond-%lf\n",GPStime.GPS_Week,GPStime.GPS_Second);
    printf("Position: X-%lf\tY-%lf\tZ-%lf\n",X,Y,Z);
    printf("Velocity: X-%lf\tY-%lf\tZ-%lf\n",X_Dot,Y_Dot,Z_Dot);
    printf("Clkdelta: %.9lf\t Clkdelta_Dot: %.15lf\n",Tmp.Clkdelta,Tmp.Clkdelta_Dot);
    printf("Deltat: %lf\n", Psr / C_LIGHT);


    return Tmp;
}
GPS_Position GPS_PVT(const Novatel_Message_GPSEPHEM &GPSEPHEM, Novatel_Data &Novatel_Data, GPST Positioning_Time, double Psr,double Rou)
{
    double deltat=Positioning_Time-GPST(GPSEPHEM.Week,GPSEPHEM.Toe);
    double deltat_corr= deltat - Psr / C_LIGHT;
    double n0 = 0;//平均运动速率(未修正)
    double n = 0;//平均运动速率(修正)
    n0 = std::sqrt(GPS_GM) / GPSEPHEM.sqrtA / GPSEPHEM.sqrtA / GPSEPHEM.sqrtA;
    n = n0 + GPSEPHEM.Delta_N;

    double M_cur = GPSEPHEM.M0 + n * deltat_corr;//计算平近点角

    double E_cur, E_tmp;//偏近点角
    E_cur = M_cur;
    do {
        E_tmp = E_cur;
        E_cur = E_tmp - (E_tmp - GPSEPHEM.e * std::sin(E_tmp) - M_cur) / (1 - GPSEPHEM.e * std::cos(E_tmp));
    } while (std::fabs(E_cur - E_tmp) > 1e-12);

    //钟差改正
    double t_corr=GPSEPHEM.sa0+GPSEPHEM.sa1*deltat+GPSEPHEM.sa2*deltat*deltat
                  -GPSEPHEM.Tgd+DELTAT_CORR_F*GPSEPHEM.e*GPSEPHEM.sqrtA*sin(E_cur);
    deltat_corr=deltat_corr-t_corr;//经过这一步修正后，计算的是卫星信号发射时刻的卫星坐标

    //计算真近点角
    double v_cur = atan2(std::sqrt(1 - GPSEPHEM.e * GPSEPHEM.e) * std::sin(E_cur), std::cos(E_cur) - GPSEPHEM.e);

    //计算升交距角（未经改正的）
    double u_cur_nocorr = GPSEPHEM.omega + v_cur;
    //计算卫星向径（未经改正的）
    double r_cur_nocorr = GPSEPHEM.sqrtA * GPSEPHEM.sqrtA * (1 - std::cos(E_cur) * GPSEPHEM.e);

    //计算摄动改正项
    double u_corr = GPSEPHEM.Cus * std::sin(2 * u_cur_nocorr) + GPSEPHEM.Cuc * std::cos(2 * u_cur_nocorr);
    double r_corr = GPSEPHEM.Crs * std::sin(2 * u_cur_nocorr) + GPSEPHEM.Crc * std::cos(2 * u_cur_nocorr);
    double i_corr = GPSEPHEM.Cis * std::sin(2 * u_cur_nocorr) + GPSEPHEM.Cic * std::cos(2 * u_cur_nocorr);

    //进行摄动改正
    double u_cur = u_cur_nocorr + u_corr;
    double r_cur = r_cur_nocorr + r_corr;
    double i_cur = GPSEPHEM.I0 + GPSEPHEM.I_Dot * deltat_corr + i_corr;

    //计算卫星在轨道平面坐标系（xoy）中的位置
    double x_cur = r_cur * std::cos(u_cur);
    double y_cur = r_cur * std::sin(u_cur);

    //计算观测瞬间升交点经度（大地经度=赤经减去GAST）
    //t时刻升交点赤经
    //double OMEGA_cur=GPSEPHEM.OMEGA0+GPSEPHEM.OMEGADOT*deltat;

    //t时刻大地经度
    double Longitude_cur = GPSEPHEM.OMEGA0 + GPSEPHEM.OMEGADOT * deltat_corr - (deltat_corr + GPSEPHEM.Toe) * GPS_OMEGAE;

    //卫星在ECEF瞬时地心地固坐标系下的坐标
    double X = x_cur * std::cos(Longitude_cur) - y_cur * std::cos(i_cur) * std::sin(Longitude_cur);
    double Y = x_cur * std::sin(Longitude_cur) + y_cur * std::cos(i_cur) * std::cos(Longitude_cur);
    double Z = y_cur * std::sin(i_cur);
    {//地球自转校正（Sagnac效应）
        double Phi_Z = GPS_OMEGAE * (Rou / C_LIGHT);

        double Pos_[3] = {X, Y, Z};
        double Tz[9] = {std::cos(Phi_Z), std::sin(Phi_Z), 0,
                        -1.0 * std::sin(Phi_Z), std::cos(Phi_Z), 0,
                        0, 0, 1};
        Matrix T_Z(3, 3, Tz);

        Matrix Position(3, 1, Pos_);
        Matrix Position_Sagnac_corr(3, 1);
        Position_Sagnac_corr = T_Z * Position;

        X = Position_Sagnac_corr.My_Matrix(0, 0);
        Y = Position_Sagnac_corr.My_Matrix(1, 0);
        Z = Position_Sagnac_corr.My_Matrix(2, 0);
    }

    // 速度计算
    double E_cur_Dot=n/(1-GPSEPHEM.e*cos(E_cur));
    double Phi_cur_Dot= std::sqrt((1+GPSEPHEM.e)/(1-GPSEPHEM.e)) * (std::cos(v_cur / 2) / std::cos(E_cur / 2) )
                        * (std::cos(v_cur / 2) / std::cos(E_cur / 2)) * E_cur_Dot;
    double u_k_Dot=2*(GPSEPHEM.Cus*cos(2*u_cur_nocorr)-GPSEPHEM.Cuc*sin(2*u_cur_nocorr))*Phi_cur_Dot+Phi_cur_Dot;
    double r_k_Dot=GPSEPHEM.A*GPSEPHEM.e*std::sin(E_cur)*E_cur_Dot+
                   2*(GPSEPHEM.Crs*cos(2*u_cur_nocorr)-GPSEPHEM.Crc*sin(2*u_cur_nocorr))*Phi_cur_Dot;
    double I_k_Dot=GPSEPHEM.I_Dot+2*(GPSEPHEM.Cis*cos(2*u_cur_nocorr)-GPSEPHEM.Cic*sin(2*u_cur_nocorr))*Phi_cur_Dot;
    double OMEGA_k_Dot=GPSEPHEM.OMEGADOT-GPS_OMEGAE;
    //轨道平面坐标导数
    double x_k_Dot=r_k_Dot*std::cos(u_cur)-r_cur*u_k_Dot*std::sin(u_cur);
    double y_k_Dot=r_k_Dot*std::sin(u_cur)+r_cur*u_k_Dot*std::cos(u_cur);

    //卫星速度
    double R_Array[12]={
            std::cos(Longitude_cur), -1.0*std::sin(Longitude_cur)* std::cos(i_cur),
            -1.0*(x_cur*std::sin(Longitude_cur)+y_cur*std::cos(Longitude_cur)*std::cos(i_cur)),y_cur*sin(Longitude_cur)*sin(i_cur),
            sin(Longitude_cur),std::cos(Longitude_cur)*std::cos(i_cur),
            (x_cur*std::cos(Longitude_cur)-y_cur*std::sin(Longitude_cur)*std::cos(i_cur)),y_cur*std::cos(Longitude_cur)*std::sin(i_cur),
            0, std::sin(i_cur),0,y_cur*std::cos(i_cur)
    };
    double Dot_Array[4]={
            x_k_Dot,y_k_Dot,OMEGA_k_Dot,I_k_Dot
    };
    Matrix R_Matrix(3,4,R_Array);
    Matrix Dot_Vector(4,1,Dot_Array);
    Matrix Pos_Vector=R_Matrix*Dot_Vector;
    double X_Dot=Pos_Vector.My_Matrix(0,0);
    double Y_Dot=Pos_Vector.My_Matrix(1,0);;
    double Z_Dot=Pos_Vector.My_Matrix(2,0);


    Cartesian_XYZ_Coord Pos(X,Y,Z);
    GPST GPStime(GPSEPHEM.Week,GPSEPHEM.Toe+deltat_corr);
    GPS_Position Tmp;
    Tmp.GPS_Time=GPStime;
    Tmp.Position=Pos;
    Tmp.v_X=X_Dot;
    Tmp.v_Y=Y_Dot;
    Tmp.v_Z=Z_Dot;
    Tmp.Clkdelta=t_corr;
    Tmp.Clkdelta_Dot=GPSEPHEM.sa1+2*GPSEPHEM.sa2*deltat+DELTAT_CORR_F*GPSEPHEM.e*GPSEPHEM.sqrtA* cos(E_cur)*E_cur_Dot;
    Novatel_Data.GPS_Position[GPSEPHEM.PRN].push_back(Tmp);

    printf("PRN:G%d\n",GPSEPHEM.PRN);
    printf("GPST: Week-%d\tSecond-%lf\n",GPStime.GPS_Week,GPStime.GPS_Second);
    printf("Position: X-%lf\tY-%lf\tZ-%lf\n",X,Y,Z);
    printf("Velocity: X-%lf\tY-%lf\tZ-%lf\n",X_Dot,Y_Dot,Z_Dot);
    printf("Clkdelta: %.9lf\t Clkdelta_Dot: %.15lf\n",Tmp.Clkdelta,Tmp.Clkdelta_Dot);
    printf("Deltat: %lf\n", Psr / C_LIGHT);


    return Tmp;
}
//BDS卫星位置解算
void BDS_PVT(const Novatel_Message_BDSEPHEMERIS& BDSEPHEM,Novatel_Data& Novatel_Data)
{
    int deltat=-1800-14;//BDS广播星历间隔1小时
    int step=300;//解算步长
    bool GEO_Flag=false;
    if (  BDSEPHEM.Satellite_ID==1||BDSEPHEM.Satellite_ID==2||BDSEPHEM.Satellite_ID==3
          ||BDSEPHEM.Satellite_ID==4||BDSEPHEM.Satellite_ID==5||BDSEPHEM.Satellite_ID==59
          ||BDSEPHEM.Satellite_ID==60||BDSEPHEM.Satellite_ID==61||BDSEPHEM.Satellite_ID==62)
    { GEO_Flag = true; }
    while(deltat<1800)
    {

        double n0 = 0;//平均运动速率(未修正)
        double n = 0;//平均运动速率(修正)
        n0 = std::sqrt(BDS_GM) / BDSEPHEM.Root_A / BDSEPHEM.Root_A / BDSEPHEM.Root_A;
        n = n0 + BDSEPHEM.Delta_N;

        double M_cur = BDSEPHEM.M0 + n * deltat;//计算平近点角

        double E_cur, E_tmp;//偏近点角
        E_cur = M_cur;
        do {
            E_tmp = E_cur;
            E_cur = E_tmp - (E_tmp - BDSEPHEM.e * sin(E_tmp) - M_cur) / (1 - BDSEPHEM.e * cos(E_tmp));
        } while (fabs(E_cur - E_tmp) > 1e-12);


        //钟差改正
        double t_corr=BDSEPHEM.sa0+BDSEPHEM.sa1*deltat+BDSEPHEM.sa2*deltat*deltat
                      +DELTAT_CORR_F*BDSEPHEM.e*BDSEPHEM.Root_A*sin(E_cur);
//        double deltat_corr=deltat-t_corr;
        double deltat_corr=deltat;

        //计算真近点角
        double f_cur = atan2(sqrt(1 - BDSEPHEM.e * BDSEPHEM.e) * sin(E_cur), cos(E_cur) - BDSEPHEM.e);

        //计算升交距角（未经改正的）
        double u_cur_nocorr = BDSEPHEM.omega + f_cur;
        //计算卫星向径（未经改正的）
        double r_cur_nocorr =  BDSEPHEM.A * (1 - cos(E_cur) * BDSEPHEM.e);

        //计算摄动改正项
        double u_corr = BDSEPHEM.Cus * sin(2 * u_cur_nocorr) + BDSEPHEM.Cuc * cos(2 * u_cur_nocorr);
        double r_corr = BDSEPHEM.Crs * sin(2 * u_cur_nocorr) + BDSEPHEM.Crc * cos(2 * u_cur_nocorr);
        double i_corr = BDSEPHEM.Cis * sin(2 * u_cur_nocorr) + BDSEPHEM.Cic * cos(2 * u_cur_nocorr);

        //进行摄动改正
        double u_cur = u_cur_nocorr + u_corr;
        double r_cur = r_cur_nocorr + r_corr;
        double i_cur = BDSEPHEM.I0 + BDSEPHEM.I_Dot * deltat_corr + i_corr;


        //计算卫星在轨道平面坐标系（xoy）中的位置
        double x_cur = r_cur * cos(u_cur);
        double y_cur = r_cur * sin(u_cur);

        //计算观测瞬间升交点经度（大地经度=赤经减去GAST）
        //t时刻升交点赤经
        //double OMEGA_cur=BDSEPHEM.OMEGA0+BDSEPHEM.OMEGADOT*deltat;

        //t时刻大地经度
        //double Longitude_cur = BDSEPHEM.OMEGA0 + BDSEPHEM.OMEGADOT * deltat - (deltat+BDSEPHEM.TOE) * omega_e;
        double Longitude_cur = BDSEPHEM.OMEGA0 + (BDSEPHEM.OMEGADOT - BDS_OMEGAE) * deltat_corr - BDS_OMEGAE * (BDSEPHEM.Toe);

        //GEO卫星单独处理
        if(GEO_Flag)
        {
            Longitude_cur = BDSEPHEM.OMEGA0 + BDSEPHEM.OMEGADOT * deltat_corr - BDS_OMEGAE * (BDSEPHEM.Toe);
        }

        //卫星在瞬时地心地固坐标系下的坐标
        double X = x_cur * cos(Longitude_cur) - y_cur * cos(i_cur) * sin(Longitude_cur);
        double Y = x_cur * sin(Longitude_cur) + y_cur * cos(i_cur) * cos(Longitude_cur);
        double Z = y_cur * sin(i_cur);

//        double xp = 0.0634 * 3.1415926/648000.0;
//        double yp = 0.3316 * 3.1415926/648000.0;
//
//        xp=0;
//        yp=0;
//
//        double X_Polar_corr = X + xp * Z;
//        double Y_Polar_corr = Y - yp * Z;
//        double Z_Polar_corr = - xp * X + yp * Y + Z;

        //GEO卫星单独处理
        if(GEO_Flag)
        {
            double f=-5*PI/180;
            double X_GEO_corr,Y_GEO_corr,Z_GEO_corr;

            X_GEO_corr= X;
            Y_GEO_corr= cos(f)*Y+ sin(f)*Z;
            Z_GEO_corr= -1.0*sin(f)*Y+ cos(f)*Z;

            X=X_GEO_corr;
            Y=Y_GEO_corr;
            Z=Z_GEO_corr;
        }


        // 速度计算
        double E_cur_Dot=n/(1-BDSEPHEM.e*cos(E_cur));
        double Phi_cur_Dot= sqrt((1+BDSEPHEM.e)/(1-BDSEPHEM.e))*( cos(f_cur/2)/ cos(E_cur/2) )
                            *( cos(f_cur/2)/ cos(E_cur/2) )*E_cur_Dot;
        double u_k_Dot=2*(BDSEPHEM.Cus*cos(2*u_cur_nocorr)-BDSEPHEM.Cuc*sin(2*u_cur_nocorr))*Phi_cur_Dot+Phi_cur_Dot;
        double r_k_Dot=BDSEPHEM.A*BDSEPHEM.e* sin(E_cur)* sin(E_cur_Dot)+
                       2*(BDSEPHEM.Crs*cos(2*u_cur_nocorr)-BDSEPHEM.Crc*sin(2*u_cur_nocorr))*Phi_cur_Dot;
        double I_k_Dot=BDSEPHEM.I_Dot+2*(BDSEPHEM.Cis*cos(2*u_cur_nocorr)-BDSEPHEM.Cic*sin(2*u_cur_nocorr))*Phi_cur_Dot;
        double OMEGA_k_Dot=BDSEPHEM.OMEGADOT-BDS_OMEGAE;
        //轨道平面坐标导数
        double x_k_Dot=r_k_Dot*cos(u_cur)-r_cur*u_k_Dot* sin(u_cur);
        double y_k_Dot=r_k_Dot*sin(u_cur)+r_cur*u_k_Dot* cos(u_cur);

        //卫星速度
        double X_Dot= cos(Longitude_cur)*x_k_Dot + (-1* sin(Longitude_cur))* cos(i_cur)*y_k_Dot
                      + -1 *(x_cur* sin(Longitude_cur)+y_cur* cos(Longitude_cur)* cos(i_cur))*OMEGA_k_Dot
                      +y_cur* sin(Longitude_cur)* sin(i_cur)*I_k_Dot;
        double Y_Dot=sin(Longitude_cur)*x_k_Dot + (cos(Longitude_cur))* cos(i_cur)*y_k_Dot
                     + (x_cur* cos(Longitude_cur)-y_cur* sin(Longitude_cur)* cos(i_cur))*OMEGA_k_Dot
                     +y_cur* cos(Longitude_cur)* sin(i_cur)*I_k_Dot;

        double Z_Dot= sin(i_cur)*y_k_Dot+y_cur* cos(i_cur)*I_k_Dot;

        Cartesian_XYZ_Coord Pos(X,Y,Z);
        BDT BDStime(BDSEPHEM.Week,BDSEPHEM.Toe+deltat);
        BDS_Position Tmp;
        Tmp.BDS_Time=BDStime;
        Tmp.Position=Pos;
        Tmp.v_X=X_Dot;
        Tmp.v_Y=Y_Dot;
        Tmp.v_Z=Z_Dot;
        Tmp.Clkdelta=t_corr;
        Tmp.Clkdelta_Dot=BDSEPHEM.sa1+2*BDSEPHEM.sa2*deltat+DELTAT_CORR_F*BDSEPHEM.e*BDSEPHEM.Root_A* cos(E_cur)*E_cur_Dot;
        Novatel_Data.BDS_Position[BDSEPHEM.Satellite_ID].push_back(Tmp);

//        printf("PRN:%d\n",BDSEPHEM.Satellite_ID);
//        printf("BDST: Week-%d\tSecond-%d\n",BDStime.BDS_Week,BDStime.BDS_Second);
//        GPST GPStime= BDT2GPST(BDStime);
//        printf("GPST: Week-%d\tSecond-%d\n",GPStime.GPS_Week,GPStime.GPS_Second);
//        printf("Position: X-%lf\tY-%lf\tZ-%lf\n",X,Y,Z);
//        printf("Velocity: X-%lf\tY-%lf\tZ-%lf\n",X_Dot,Y_Dot,Z_Dot);
//        printf("Clkdelta: %.9lf\t Clkdelta_Dot: %.15lf\n",Tmp.Clkdelta,Tmp.Clkdelta_Dot);



//        double refTime=0;//2019 12 01 00 00 00 周内秒为0
//        double t_cur = deltat_corr + BDSEPHEM.TOE;
//        if(BDSEPHEM.sPRN[0]=='C')
//        {
//            t_cur+=14;
//        }
//        double dt    = t_cur - refTime;
//        double theta = omega_e * dt;
//
//        // 把瞬时 ECEF 逆旋转到参考时刻 ECEF
//        double cos_t = std::cos(theta);
//        double sin_t = std::sin(theta);
//
//        double X_ref =  cos_t * X_Polar_corr - sin_t * Y_Polar_corr;
//        double Y_ref =  sin_t * X_Polar_corr + cos_t * Y_Polar_corr;
//        double Z_ref =  Z_Polar_corr;
        deltat+=step;
    }
}
BDS_Position BDS_PVT(const Novatel_Message_BDSEPHEMERIS &BDSEPHEM, Novatel_Data &Novatel_Data, BDT Positioning_Time, double Psr)
{
    double deltat=Positioning_Time.BDS_Second-BDSEPHEM.Toe+604800*(Positioning_Time.BDS_Week-BDSEPHEM.Week);
    double deltat_corr= deltat - Psr / C_LIGHT;//表面时修正——型号传播时间

    bool GEO_Flag = (BDSEPHEM.Satellite_ID >=1 && BDSEPHEM.Satellite_ID <=5)
                    || (BDSEPHEM.Satellite_ID >=59 && BDSEPHEM.Satellite_ID <=62);

    double n0 = 0;//平均运动速率(未修正)
    double n = 0;//平均运动速率(修正)
    n0 = std::sqrt(BDS_GM) / BDSEPHEM.Root_A / BDSEPHEM.Root_A / BDSEPHEM.Root_A;
    n = n0 + BDSEPHEM.Delta_N;
    double M_cur;
    if (GEO_Flag)
    {
        M_cur= BDSEPHEM.M0 + n * deltat;//计算平近点角
    } else{
        M_cur = BDSEPHEM.M0 + n *deltat_corr;
    }
    double E_cur, E_tmp;//偏近点角
    E_cur = M_cur;
    do {
        E_tmp = E_cur;
        E_cur = E_tmp - (E_tmp - BDSEPHEM.e * std::sin(E_tmp) - M_cur) / (1 - BDSEPHEM.e * std::cos(E_tmp));
    } while (fabs(E_cur - E_tmp) > 1e-12);

    //钟差改正
    double t_corr=BDSEPHEM.sa0+BDSEPHEM.sa1*deltat+BDSEPHEM.sa2*deltat*deltat
                  +DELTAT_CORR_F*BDSEPHEM.e*BDSEPHEM.Root_A*sin(E_cur);
    deltat_corr=deltat_corr-t_corr;//经过这一步修正后，计算的是卫星信号发射时刻的卫星坐标


    //计算真近点角
    double f_cur = std::atan2(std::sqrt(1 - BDSEPHEM.e * BDSEPHEM.e) * std::sin(E_cur), std::cos(E_cur) - BDSEPHEM.e);

    //计算升交距角（未经改正的）
    double u_cur_nocorr = BDSEPHEM.omega + f_cur;
    //计算卫星向径（未经改正的）
    double r_cur_nocorr =  BDSEPHEM.A * (1 - std::cos(E_cur) * BDSEPHEM.e);

    //计算摄动改正项
    double u_corr = BDSEPHEM.Cus * std::sin(2 * u_cur_nocorr) + BDSEPHEM.Cuc * std::cos(2 * u_cur_nocorr);
    double r_corr = BDSEPHEM.Crs * std::sin(2 * u_cur_nocorr) + BDSEPHEM.Crc * std::cos(2 * u_cur_nocorr);
    double i_corr = BDSEPHEM.Cis * std::sin(2 * u_cur_nocorr) + BDSEPHEM.Cic * std::cos(2 * u_cur_nocorr);

    //进行摄动改正
    double u_cur = u_cur_nocorr + u_corr;
    double r_cur = r_cur_nocorr + r_corr;
    double i_cur = BDSEPHEM.I0 + BDSEPHEM.I_Dot * deltat_corr + i_corr;


    //计算卫星在轨道平面坐标系（xoy）中的位置
    double x_cur = r_cur * std::cos(u_cur);
    double y_cur = r_cur * std::sin(u_cur);

    //计算观测瞬间升交点经度（大地经度=赤经减去GAST）
    //t时刻升交点赤经
    //double OMEGA_cur=BDSEPHEM.OMEGA0+BDSEPHEM.OMEGADOT*deltat;

    //t时刻大地经度
    //double Longitude_cur = BDSEPHEM.OMEGA0 + BDSEPHEM.OMEGADOT * deltat - (deltat+BDSEPHEM.TOE) * omega_e;
    double Longitude_cur = BDSEPHEM.OMEGA0 + (BDSEPHEM.OMEGADOT - BDS_OMEGAE) * deltat_corr - BDS_OMEGAE * (BDSEPHEM.Toe);

    //GEO卫星单独处理
    if(GEO_Flag)
    {
        Longitude_cur = BDSEPHEM.OMEGA0 + BDSEPHEM.OMEGADOT * deltat_corr - BDS_OMEGAE * (BDSEPHEM.Toe);
    }

    //卫星在瞬时地心地固坐标系下的坐标
    double X = x_cur * cos(Longitude_cur) - y_cur * cos(i_cur) * sin(Longitude_cur);
    double Y = x_cur * sin(Longitude_cur) + y_cur * cos(i_cur) * cos(Longitude_cur);
    double Z = y_cur * sin(i_cur);
//    {
//        double xp=0.11299*PI/3600/180;
//        double yp=0.19059*PI/3600/180;
//
//
//        double Pos[3] = {X, Y, Z};
//        double T[9] = {std::cos(xp) , std::sin(xp)*std::sin(yp), std::sin(xp)*std::cos(yp),
//                       std::sin(yp)*std::sin(xp), std::cos(yp), -1.0*std::sin(yp)*std::cos(xp),
//                -1.0*std::sin(xp), std::sin(yp)*std::cos(xp), std::cos(xp)*std::cos(yp)};
//        Matrix T_Matrix(3, 3, T);
//
//        Matrix Position(3, 1, Pos);
//        Matrix Position_Polar_corr(3, 1);
//        Position_Polar_corr = T_Matrix * Position;
//
//        X = Position_Polar_corr.My_Matrix(0, 0);
//        Y = Position_Polar_corr.My_Matrix(1, 0);
//        Z = Position_Polar_corr.My_Matrix(2, 0);
//    }
    // 速度计算
    double E_cur_Dot=n/(1-BDSEPHEM.e*cos(E_cur));
    double Phi_cur_Dot= sqrt(1-BDSEPHEM.e*BDSEPHEM.e)/(1-BDSEPHEM.e* cos(E_cur))*E_cur_Dot;
    double u_k_Dot=2*(BDSEPHEM.Cus*cos(2*u_cur_nocorr)-BDSEPHEM.Cuc*sin(2*u_cur_nocorr))*Phi_cur_Dot+Phi_cur_Dot;
    double r_k_Dot=BDSEPHEM.A*BDSEPHEM.e* sin(E_cur)* E_cur_Dot+
                   2*(BDSEPHEM.Crs*cos(2*u_cur_nocorr)-BDSEPHEM.Crc*sin(2*u_cur_nocorr))*Phi_cur_Dot;
    double I_k_Dot=BDSEPHEM.I_Dot+2*(BDSEPHEM.Cis*cos(2*u_cur_nocorr)-BDSEPHEM.Cic*sin(2*u_cur_nocorr))*Phi_cur_Dot;
    double OMEGA_k_Dot=BDSEPHEM.OMEGADOT-BDS_OMEGAE;
    if(GEO_Flag) {
        OMEGA_k_Dot = BDSEPHEM.OMEGADOT;
    }
    //轨道平面坐标导数
    double x_k_Dot=r_k_Dot*cos(u_cur)-r_cur*u_k_Dot* sin(u_cur);
    double y_k_Dot=r_k_Dot*sin(u_cur)+r_cur*u_k_Dot* cos(u_cur);

    //卫星速度
    double X_Dot= cos(Longitude_cur)*x_k_Dot
                  - sin(Longitude_cur)*(y_k_Dot* cos(i_cur)-Z*I_k_Dot)
                  - Y*OMEGA_k_Dot;

    double Y_Dot= sin(Longitude_cur)*x_k_Dot
                  + cos(Longitude_cur)*(y_k_Dot* cos(i_cur)-Z*I_k_Dot)
                  + X*OMEGA_k_Dot;

    double Z_Dot= sin(i_cur)*y_k_Dot+y_cur* cos(i_cur)*I_k_Dot;


    //GEO卫星单独处理
    if(GEO_Flag)
    {

        double Phi_X=-5*PI/180;
        double Phi_Z=BDS_OMEGAE*(deltat);

        double Pos[3]={X,Y,Z};
        double Vel[3]={X_Dot,Y_Dot,Z_Dot};
        double Tx[9]={1,0,0,
                      0,std::cos(Phi_X),std::sin(Phi_X),
                      0,-1.0*std::sin(Phi_X),std::cos(Phi_X)};
        double Tz[9]={std::cos(Phi_Z),std::sin(Phi_Z),0,
                      -1.0*std::sin(Phi_Z),std::cos(Phi_Z),0,
                      0,0,1};
        double Tz_Dot[9]={BDS_OMEGAE*-1.0*std::sin(Phi_Z),BDS_OMEGAE*std::cos(Phi_Z),0,
                          BDS_OMEGAE*-1.0*std::cos(Phi_Z),BDS_OMEGAE*-1.0*std::sin(Phi_Z),0,
                          0,0,0};


        Matrix T_X(3,3,Tx);
        Matrix T_Z(3,3,Tz);
        Matrix T_Z_Dot(3,3,Tz_Dot);
        Matrix Velocity(3,1,Vel);
        Matrix Position(3,1,Pos);
        Matrix Velocity_GEO_corr(3,1);
        Matrix Position_GEO_corr(3,1);

        Position_GEO_corr=T_Z*T_X*Position;
        Velocity_GEO_corr=T_Z*T_X*Velocity+T_Z_Dot*T_X*Position;

        X=Position_GEO_corr.My_Matrix(0,0);
        Y=Position_GEO_corr.My_Matrix(1,0);
        Z=Position_GEO_corr.My_Matrix(2,0);
        X_Dot=Velocity_GEO_corr.My_Matrix(0,0);
        Y_Dot=Velocity_GEO_corr.My_Matrix(1,0);
        Z_Dot=Velocity_GEO_corr.My_Matrix(2,0);

        double Phi_Sagnac_Z = BDS_OMEGAE * Psr / C_LIGHT;

        double Tz_Sagnac[9] = {std::cos(Phi_Sagnac_Z), std::sin(Phi_Sagnac_Z), 0,
                               -1.0 * std::sin(Phi_Sagnac_Z), std::cos(Phi_Sagnac_Z), 0,
                               0, 0, 1};
        Matrix T_Z_Sagnac(3, 3, Tz_Sagnac);

        Matrix Position_Sagnac_corr(3, 1);
        Position_Sagnac_corr = T_Z_Sagnac * Position_GEO_corr;

        X = Position_Sagnac_corr.My_Matrix(0, 0);
        Y = Position_Sagnac_corr.My_Matrix(1, 0);
        Z = Position_Sagnac_corr.My_Matrix(2, 0);


    }
    else{
        double Phi_Sagnac_Z = BDS_OMEGAE * (Psr / C_LIGHT+t_corr);
//        double xp=0.11299*PI/3600/180;
//        double yp=0.19059*PI/3600/180;


        double Pos[3] = {X, Y, Z};
        double Tz_Sagnac[9] = {std::cos(Phi_Sagnac_Z), std::sin(Phi_Sagnac_Z), 0,
                               -1.0 * std::sin(Phi_Sagnac_Z), std::cos(Phi_Sagnac_Z), 0,
                               0, 0, 1};
//        double Tz_Sagnac[9] = {1, BDS_OMEGAE * (Psr / C_LIGHT), 0,
//                               -1.0 * BDS_OMEGAE * (Psr / C_LIGHT), 1, 0,
//                               0, 0, 1};
//        double T[9] = {std::cos(xp) , std::sin(xp)*std::sin(yp), std::sin(xp)*std::cos(yp),
//                       std::sin(yp)*std::sin(xp), std::cos(yp), -1.0*std::sin(yp)*std::cos(xp),
//                       -1.0*std::sin(xp), std::sin(yp)*std::cos(xp), std::cos(xp)*std::cos(yp)};
//        Matrix T_Polar(3, 3, T);
        Matrix T_Z_Sagnac(3, 3, Tz_Sagnac);

        Matrix Position(3, 1, Pos);
        Matrix Position_Sagnac_corr(3, 1);
        Matrix Position_Polar_corr(3, 1);

        Position_Sagnac_corr = T_Z_Sagnac * Position;
//        Position_Polar_corr =T_Polar  * Position_Sagnac_corr;

        X = Position_Sagnac_corr.My_Matrix(0, 0);
        Y = Position_Sagnac_corr.My_Matrix(1, 0);
        Z = Position_Sagnac_corr.My_Matrix(2, 0);
//
//        X = Position_Polar_corr.My_Matrix(0, 0);
//        Y = Position_Polar_corr.My_Matrix(1, 0);
//        Z = Position_Polar_corr.My_Matrix(2, 0);

    }



//        double deltat_corr=deltat;

    Cartesian_XYZ_Coord Pos(X,Y,Z);
    BDT BDStime(BDSEPHEM.Week,BDSEPHEM.Toe+deltat_corr);
    BDS_Position Tmp;
    Tmp.BDS_Time=BDStime;
    Tmp.Position=Pos;
    Tmp.v_X=X_Dot;
    Tmp.v_Y=Y_Dot;
    Tmp.v_Z=Z_Dot;
    Tmp.Clkdelta=t_corr;
    Tmp.Clkdelta_Dot=BDSEPHEM.sa1+2*BDSEPHEM.sa2*deltat+DELTAT_CORR_F*BDSEPHEM.e*BDSEPHEM.Root_A* cos(E_cur)*E_cur_Dot;
    Novatel_Data.BDS_Position[BDSEPHEM.Satellite_ID].push_back(Tmp);

    printf("PRN:C%d\n",BDSEPHEM.Satellite_ID);
    printf("BDST: Week-%d\tSecond-%lf\n",BDStime.BDS_Week,BDStime.BDS_Second);
    GPST GPStime= BDT2GPST(BDStime);
    printf("GPST: Week-%d\tSecond-%lf\n",GPStime.GPS_Week,GPStime.GPS_Second);
    printf("Position: X-%lf\tY-%lf\tZ-%lf\n",X,Y,Z);
    printf("Velocity: X-%lf\tY-%lf\tZ-%lf\n",X_Dot,Y_Dot,Z_Dot);
    printf("Clkdelta: %.9lf\t Clkdelta_Dot: %.15lf\n",Tmp.Clkdelta,Tmp.Clkdelta_Dot);
    printf("Deltat: %lf\n", Psr / C_LIGHT);
    return Tmp;
}

//卫星位置 std::sort按照时序排序
bool GPS_Position_Compare(const GPS_Position& a, const GPS_Position& b)
{
    if(a.GPS_Time.GPS_Week!=b.GPS_Time.GPS_Week)
    {
        return a.GPS_Time.GPS_Week<b.GPS_Time.GPS_Week;
    }
    return a.GPS_Time.GPS_Second<b.GPS_Time.GPS_Second;
}
bool BDS_Position_Compare(const BDS_Position& a, const BDS_Position& b)
{
    if(a.BDS_Time.BDS_Week!=b.BDS_Time.BDS_Week)
    {
        return a.BDS_Time.BDS_Week<b.BDS_Time.BDS_Week;
    }
    return a.BDS_Time.BDS_Second<b.BDS_Time.BDS_Second;
}

//将解算结果输出到csv
void Export_Position2CSV(const std::map<uint32_t, std::vector<GPS_Position>>& data, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing.\n";
        return;
    }

    // 写CSV表头
    file << "Key,"
         << "GPST_Week,GPST_Seconds,"
         << "X,Y,Z,v_X,v_Y,v_Z,Clkdelta,Clkdelta_Dot\n";

    for (const auto& pair : data) {
        uint32_t key = pair.first;
        for (const auto& pos : pair.second) {
            file << key << ","
                 << pos.GPS_Time.GPS_Week << "," << pos.GPS_Time.GPS_Second << ","
                 << pos.Position.X << "," << pos.Position.Y << "," << pos.Position.Z << ","
                 << pos.v_X << "," << pos.v_Y << "," << pos.v_Z << ","
                 << pos.Clkdelta << "," << pos.Clkdelta_Dot << "\n";
        }
    }

    file.close();
    std::cout << "Data written to " << filename << std::endl;
}