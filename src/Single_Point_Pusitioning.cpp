//
// Created by asus on 2025/6/5.
//
#include "DATA_Novatel.h"

//Hopfield模型改正
double Hopfield(double Elevation_Angle,double Height)
{
    // 标准大气参数
    const double T0 = 288.15;      // 海平面温度(K) 15°C
    const double p0 = 1013.25;     // 海平面气压(hPa)
    const double RH0 = 0.50;       // 海平面相对湿度(小数)
    const double H0 = 0.0;         // 参考高度(m)

    if (Height > 43000.0||Height<0)
        return 0.0;

    double RH = RH0 * exp(-0.0006396 * (Height - H0));
    double p = p0 * pow(1 - 0.0000226 * (Height - H0), 5.225);
    double T = T0 - 0.0065*(Height - H0);
    double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
    double hw = 11000.0;
    double hd = 40136 + 148.72 * (T0 - 273.16);
    double Kw = 155.2e-7 * 4810 / (T * T) * e * (hw - Height);
    double Kd = 155.2e-7 * (p / T) * (hd - Height);

    double delta_Trop = Kd / std::sin(std::sqrt(Elevation_Angle * Elevation_Angle + 6.25*DEG2RAD*DEG2RAD))
                        + Kw / std::sin(std::sqrt(Elevation_Angle * Elevation_Angle + 2.25*DEG2RAD*DEG2RAD));

    return delta_Trop;
}
//单点定位及测速   (信号传播时间改正方案1） 对流层模型 Hopfield
//20250601 双频组合
void Single_Point_Pusitioning(Novatel_Data& Novatel_Data,
                              Novatel_Single_Epoch_OBSERVATION& Single_Epoch_Obs)
{
    //历元时间
    GPST Epoch_Time=Single_Epoch_Obs.GPS_Time;
    BDT Epoch_Time_BDS=GPST2BDT(Epoch_Time);

    printf("Epoch Time:GPST week-%d second-%.9lf\n",Epoch_Time.GPS_Week,Epoch_Time.GPS_Second);
    printf("Epoch Time:BDT week-%d second-%.9lf\n",Epoch_Time_BDS.BDS_Week,Epoch_Time_BDS.BDS_Second);

    //该历元卫星位置
    std::map<uint32_t,GPS_Position> GPS_PV;
    std::map<uint32_t,BDS_Position> BDS_PV;
    Receiver_Position Receiver_PV;
    //卫星星历
    std::map<uint32_t,Novatel_Message_GPSEPHEM> GPS_Ephem;
    std::map<uint32_t,Novatel_Message_BDSEPHEMERIS> BDS_Ephem;
    //IF伪距计算 卫星位置计算
    for (auto& [prn,GPS_obs] : Single_Epoch_Obs.Single_Epoch_GPS_Observation)
    {
        double Obs1=0;
        double Sigma1=9999;
        double Obs2=0;
        double Sigma2=9999;
        double Doppler=0;
        double CNO=30;
        //计算消电离层的伪距差分观测值
        for (auto obs : GPS_obs)
        {
            if(obs.Signal_Type==0||obs.Signal_Type==16)
            {
                //L1波段
                if(obs.Psr_Sigma<Sigma1){
                    Obs1 = obs.Psr;
                    Sigma1 = obs.Psr_Sigma;
                }
                if(obs.C_No>CNO){
                    Doppler = obs.Dopp * GPS_L1_LAMDA;
                    CNO = obs.C_No;
                }
            }
            if(obs.Signal_Type==5||obs.Signal_Type==9||obs.Signal_Type==17)
            {
                //L2波段

                if(obs.Psr_Sigma<Sigma2){
                    Obs2=obs.Psr;
                    Sigma2 = obs.Psr_Sigma;
                }
            }
        }
        if(Obs1==0||Obs2==0)
        {
            break;
        }
        double IF_Obs= IF_GPS_M * Obs1 + IF_GPS_N * Obs2;
        Single_Epoch_Obs.Ionospheric_Free_GPS_Observation[prn]=IF_Obs;
        Single_Epoch_Obs.Ionospheric_Free_Sigma_GPS_Observation[prn]=std::sqrt(IF_GPS_M* Sigma1*IF_GPS_M* Sigma1+IF_GPS_N * Sigma2*IF_GPS_N * Sigma2);
        Single_Epoch_Obs.Doppler_GPS_Observation[prn]=Doppler;
        Single_Epoch_Obs.Doppler_CNO_GPS_Observation[prn]=CNO;

        //计算卫星位置
        auto& GPSEPHEM=Novatel_Data.vMessage_GPSEPHEM;
        for (auto Ephem : GPSEPHEM)
        {
            if(abs(Epoch_Time-GPST(Ephem.Week,Ephem.Toe))<3600)
            {
                if(prn==Ephem.PRN)
                {
                    GPS_Ephem[prn]=Ephem;
//                    GPST Epoch_Time_Satellite_corr=Epoch_Time;
//                    Epoch_Time_Satellite_corr.GPS_Second=Epoch_Time_Satellite_corr.GPS_Second-IF_Obs/C_LIGHT;
                    GPS_Position res = GPS_PVT(Ephem, Novatel_Data, Epoch_Time, IF_Obs);
                    GPS_PV[prn] = res;
                    break;
                }
            }
        }
    }
    for (auto& [prn,BDS_obs] : Single_Epoch_Obs.Single_Epoch_BDS_Observation)
    {
        double Obs1=0;
        double Sigma1=9999;
        double Obs2=0;
        double Sigma2=9999;
        double Doppler=0;
        double CNO=30;
        for (auto obs : BDS_obs)
        {
            if(obs.Signal_Type==0||obs.Signal_Type==4||obs.Signal_Type==7)
            {
                //B1波段
                if(obs.Psr_Sigma<Sigma1){
                    Obs1 = obs.Psr;
                    Sigma1 = obs.Psr_Sigma;
                }
                if(obs.C_No>CNO){
                    Doppler = obs.Dopp * BDS_B1_LAMDA;
                    CNO = obs.C_No;
                }
                //
            }
            if(obs.Signal_Type==2||obs.Signal_Type==6)
            {
                //B3波段
                if(obs.Psr_Sigma<Sigma2){
                    Obs2=obs.Psr;
                    Sigma2 = obs.Psr_Sigma;
                }
            }
        }
        if(Obs1==0||Obs2==0)
        {
            break;
        }
//        double IF_Obs;
        double IF_Obs= IF_BDS_M * Obs1 + IF_BDS_N * Obs2;
        Single_Epoch_Obs.Ionospheric_Free_BDS_Observation[prn]=IF_Obs;
        Single_Epoch_Obs.Ionospheric_Free_Sigma_BDS_Observation[prn]=std::sqrt(IF_BDS_M* Sigma1*IF_BDS_M* Sigma1+IF_BDS_N * Sigma2*IF_BDS_N * Sigma2);
        Single_Epoch_Obs.Doppler_BDS_Observation[prn]=Doppler;
        Single_Epoch_Obs.Doppler_CNO_BDS_Observation[prn]=CNO;
        //计算卫星位置
        auto& BDSEPHEM=Novatel_Data.vMessage_BDSEPHEMERIS;
        for (auto Ephem : BDSEPHEM)
        {
            if(abs(Epoch_Time_BDS-BDT(Ephem.Week,Ephem.Toe))<1800)
            {
                if(prn==Ephem.Satellite_ID) {

                    IF_Obs+=C_LIGHT*IF_BDS_K13*Ephem.Tgd1/(1-IF_BDS_K13);//北斗B1频段的tgd改正
                    Single_Epoch_Obs.Ionospheric_Free_BDS_Observation[prn]=IF_Obs;

                    BDS_Ephem[prn]=Ephem;
//                    BDT Epoch_Time_BDS_Satellite_corr=Epoch_Time_BDS;
//                    Epoch_Time_BDS_Satellite_corr.BDS_Second=Epoch_Time_BDS_Satellite_corr.BDS_Second-IF_Obs/C_LIGHT;//此处先对时间作伪距修正，钟差修正放在了定位函数内
                    BDS_Position res = BDS_PVT(Ephem, Novatel_Data, Epoch_Time_BDS, IF_Obs);
                    BDS_PV[prn] = res;
                    break;
                }
            }
        }
    }
    const int l=BDS_PV.size()+GPS_PV.size();

    //设计矩阵\系数矩阵
    double H[1024]={0};
    //观测值
    double Z[256]={0};

    double Troposphere_corr[256]={0};
    double Rou[256]={0};

    double W[256]={0};

    double Hat_Sigma_0=0;//单位权中误差
    double PDop=0;
    //解算结果
    Matrix X_vector(5,1);
    Matrix dX_vector(5,1);

    //辅助矩阵
    double Help[5]={1,1,1,0,0};
    Matrix Help_Matrix(5,Help);
    Matrix D_X;



    //测站坐标估计值
    double X_star=0;
    double Y_star=0;
    double Z_star=0;
//    if(!Novatel_Data.SPP_Result.empty())
//    {
//        X_star=Novatel_Data.SPP_Result.back().Position.X;
//        Y_star=Novatel_Data.SPP_Result.back().Position.Y;
//        Z_star=Novatel_Data.SPP_Result.back().Position.Z;
//    }

    //接收机钟差
    double Receiver_GPS_Clkcorr=0;//2025 5 26 zhibaiyun 此处把接收机钟差融入待估参数中
    double Receiver_BDS_Clkcorr=0;
    //解算接收机位置的主迭代循环
    int cnt=0;
    while(++cnt)
    {
        printf(" Times %d Start\n",cnt);
        Cartesian_XYZ_Coord P_Receiver(X_star,Y_star,Z_star);
        int rowcnt=0;
        int obs_drop=0;
        //设计矩阵赋值
        for (auto& [prn,sat_info] : GPS_PV)
        {

            //卫星位置
            Cartesian_XYZ_Coord P_Satellite=sat_info.Position;

            ENU ENU_Satellite= XYZ2ENU(P_Satellite,P_Receiver);
            double E=ENU_Satellite.E;
            double N=ENU_Satellite.N;
            double U=ENU_Satellite.U;
            double Elevation_Angle=std::atan2(U,std::sqrt((E*E+N*N)));
            std::cout<<'G'<<prn<<' '<<Elevation_Angle*180/PI<<std::endl;
//            printf("U:%lf\nElevation_Angle:%lf\n",U,Elevation_Angle*180/PI);
            if(cnt!=1&&Elevation_Angle*180/PI<15)
            {
                obs_drop++;
                continue;
            }

            Rou[rowcnt]= std::sqrt(std::pow(P_Satellite.X-X_star,2)+std::pow(P_Satellite.Y-Y_star,2)+std::pow(P_Satellite.Z-Z_star,2));
            //为设计矩阵该行赋值
            H[rowcnt*5+0]=(X_star-P_Satellite.X)/Rou[rowcnt];
            H[rowcnt*5+1]=(Y_star-P_Satellite.Y)/Rou[rowcnt];
            H[rowcnt*5+2]=(Z_star-P_Satellite.Z)/Rou[rowcnt];
            H[rowcnt*5+3]=1;
            H[rowcnt*5+4]=0;

            //观测值

            BLH_Coord BLH_Receiver= XYZ2BLH(P_Receiver);
            Troposphere_corr[rowcnt] = Hopfield(Elevation_Angle, BLH_Receiver.H);
//            printf("C%d_Troposphere_corr:%lf\n",prn,Troposphere_corr[rowcnt]);
            //标准差倒数作为权重
            W[rowcnt]=1.0/Single_Epoch_Obs.Ionospheric_Free_Sigma_GPS_Observation[prn]
                      /Single_Epoch_Obs.Ionospheric_Free_Sigma_GPS_Observation[prn];

            Z[rowcnt]=Single_Epoch_Obs.Ionospheric_Free_GPS_Observation[prn]-Rou[rowcnt]
                      -Receiver_GPS_Clkcorr+sat_info.Clkdelta*C_LIGHT-Troposphere_corr[rowcnt];
            rowcnt++;
        }
        for (auto& [prn,sat_info] : BDS_PV)
        {
            //卫星位置
            Cartesian_XYZ_Coord P_Satellite=sat_info.Position;

            ENU ENU_Satellite= XYZ2ENU(P_Satellite,P_Receiver);
            double E=ENU_Satellite.E;
            double N=ENU_Satellite.N;
            double U=ENU_Satellite.U;

            double Elevation_Angle=std::atan2(U,std::sqrt((E*E+N*N)));
            std::cout<<'C'<<prn<<' '<<Elevation_Angle*180/PI<<std::endl;
//            printf("U:%lf\nElevation_Angle:%lf\n",U,Elevation_Angle*180/PI);
            if(cnt!=1&&Elevation_Angle*180/PI<15)
            {
                obs_drop++;
                continue;
            }

            Rou[rowcnt]= std::sqrt(std::pow(P_Satellite.X-X_star,2)+std::pow(P_Satellite.Y-Y_star,2)+std::pow(P_Satellite.Z-Z_star,2));
            //为设计矩阵该行赋值
            H[rowcnt*5+0]=(X_star-P_Satellite.X)/Rou[rowcnt];
            H[rowcnt*5+1]=(Y_star-P_Satellite.Y)/Rou[rowcnt];
            H[rowcnt*5+2]=(Z_star-P_Satellite.Z)/Rou[rowcnt];
            H[rowcnt*5+3]=0;
            H[rowcnt*5+4]=1;

            //观测值

            BLH_Coord BLH_Receiver= XYZ2BLH(P_Receiver);
            Troposphere_corr[rowcnt] = Hopfield(Elevation_Angle, BLH_Receiver.H);

//            printf("C%d_Troposphere_corr:%lf\n",prn,Troposphere_corr[rowcnt]);
            //标准差倒数作为权重
            W[rowcnt]=1.0/Single_Epoch_Obs.Ionospheric_Free_Sigma_BDS_Observation[prn]
                      /Single_Epoch_Obs.Ionospheric_Free_Sigma_BDS_Observation[prn];

            Z[rowcnt]=Single_Epoch_Obs.Ionospheric_Free_BDS_Observation[prn]-Rou[rowcnt]
                      -Receiver_BDS_Clkcorr+sat_info.Clkdelta*C_LIGHT-Troposphere_corr[rowcnt];
            rowcnt++;
        }

        if(l-obs_drop-5<0)
        {
            printf("观测数量不足\n");
            break;
        }

        Matrix H_matrix(l-obs_drop,5,H);

        //权矩阵，等权时实际上可以忽略
        Matrix P(l-obs_drop, W);

        //线性化后的 观测值向量
        Matrix Z_matrix(l-obs_drop,1,Z);
//        Z_matrix.print();
        //协因数矩阵
        Matrix Q=(H_matrix.transpose()*P*H_matrix).inverse();

        dX_vector=Q*H_matrix.transpose()*P*Z_matrix;

        X_vector=X_vector+dX_vector;
        X_star=X_vector.My_Matrix(0,0);
        Y_star=X_vector.My_Matrix(1,0);
        Z_star=X_vector.My_Matrix(2,0);
        Receiver_GPS_Clkcorr=X_vector.My_Matrix(3,0);
        Receiver_BDS_Clkcorr=X_vector.My_Matrix(4,0);

        printf("X:%lf\tY:%lf\tZ:%lf\nReceiver_GPS_Clkcorr:%lf\tReceiver_BDS_Clkcorr:%lf\n",X_star,Y_star,Z_star,
               Receiver_GPS_Clkcorr,Receiver_BDS_Clkcorr);
        printf("%s\n",std::string(66, '-').c_str());


        if((dX_vector.transpose()*Help_Matrix*dX_vector).My_Matrix(0,0)<1e-6)
        {
            printf("Z_matrix:\n");
            Z_matrix.print();

            BLH_Coord BLH_Receiver= XYZ2BLH(P_Receiver);
            printf("X:%lf\tY:%lf\tZ:%lf\nReceiver_GPS_Clkcorr:%lf\tReceiver_BDS_Clkcorr:%lf\n",X_star,Y_star,Z_star,
                   Receiver_GPS_Clkcorr,Receiver_BDS_Clkcorr);
            printf("B:%lf\tL:%lf\tH:%lf\n",BLH_Receiver.B*RAD2DEG,BLH_Receiver.L*RAD2DEG,BLH_Receiver.H);
            Matrix V=H_matrix*dX_vector-Z_matrix;
            Hat_Sigma_0=std::sqrt((V.transpose()*P*V).My_Matrix(0,0)/(l-obs_drop-5));
            printf("Hat_Sigma_0:%lf\n",Hat_Sigma_0);
            D_X=Q*Hat_Sigma_0*Hat_Sigma_0;
            D_X.print();
            Matrix Tmp=(H_matrix.transpose()*H_matrix).inverse();
            PDop=std::sqrt(Tmp.My_Matrix(0,0)
                           +Tmp.My_Matrix(1,1)
                           +Tmp.My_Matrix(2,2));
            printf("PDop:%lf\n",PDop);
            printf("%s\n",std::string(66, '-').c_str());

            Receiver_PV.Position=Cartesian_XYZ_Coord(X_star,Y_star,Z_star);
            break;
        }
        if(cnt>9)
        {
            printf("迭代解算10次不收敛，请检查数据\n");
            break;
        }

//        //接收机钟差修正伪距后再计算一次卫星位置（信号传播改正方案2 可选）
//        GPS_PV.clear();
//        BDS_PV.clear();
//        rowcnt=0;
//        for (auto &[prn,Ephem]:GPS_Ephem) {
//            GPS_PV[prn]= GPS_PVT(Ephem,Novatel_Data,Epoch_Time,
//                                 Single_Epoch_Obs.Ionospheric_Free_GPS_Observation[prn],Rou[rowcnt]);
////            GPS_PV[prn]= GPS_PVT(Ephem,Novatel_Data,Epoch_Time,Rou[rowcnt]+Receiver_GPS_Clkcorr);
//            rowcnt++;
//        }
//        for (auto &[prn,Ephem]:BDS_Ephem) {
//            BDS_PV[prn]= BDS_PVT(Ephem,Novatel_Data,Epoch_Time_BDS,
//                                 Single_Epoch_Obs.Ionospheric_Free_BDS_Observation[prn]);
////            BDS_PV[prn]= BDS_PVT(Ephem,Novatel_Data,Epoch_Time_BDS,Rou[rowcnt]+Receiver_BDS_Clkcorr);
//            rowcnt++;
//        }

    }

    //单点测速
    Matrix Vel_vector(4,1);
    double Hat_Sigma_Vel_0=0;//单位权中误差
    Matrix D_Vel;
    //设计矩阵\系数矩阵
    double H_Vel[1024]={0};
    //观测值
    double Z_Vel[256]={0};
    double Rou_Vel[256]={0};
    //权 信噪比
    double W_Vel[256]={0};
    for(int i=1;i>0;i--){
        Cartesian_XYZ_Coord P_Receiver(X_star,Y_star,Z_star);
        int rowcnt=0;
        int obs_drop=0;
        //设计矩阵赋值
        for (auto& [prn,sat_info] : GPS_PV)
        {

            //卫星位置
            Cartesian_XYZ_Coord P_Satellite=sat_info.Position;

            ENU ENU_Satellite= XYZ2ENU(P_Satellite,P_Receiver);
            double E=ENU_Satellite.E;
            double N=ENU_Satellite.N;
            double U=ENU_Satellite.U;
            double Elevation_Angle=std::atan2(U,std::sqrt((E*E+N*N)));
//            std::cout<<'G'<<prn<<' '<<Elevation_Angle*180/PI<<std::endl;
//            printf("U:%lf\nElevation_Angle:%lf\n",U,Elevation_Angle*180/PI);
            if(cnt!=1&&Elevation_Angle*180/PI<15)
            {
                obs_drop++;
                continue;
            }

            Rou_Vel[rowcnt]= std::sqrt(std::pow(P_Satellite.X-X_star,2)+std::pow(P_Satellite.Y-Y_star,2)+std::pow(P_Satellite.Z-Z_star,2));
            //为设计矩阵该行赋值
            H_Vel[rowcnt*4+0]=(P_Satellite.X-X_star)/Rou_Vel[rowcnt];
            H_Vel[rowcnt*4+1]=(P_Satellite.Y-Y_star)/Rou_Vel[rowcnt];
            H_Vel[rowcnt*4+2]=(P_Satellite.Z-Z_star)/Rou_Vel[rowcnt];
            H_Vel[rowcnt*4+3]=1;

            //观测值
//            W[rowcnt]=Single_Epoch_Obs.Doppler_CNO_GPS_Observation[prn]/30;
//            W_Vel[rowcnt]=std::sin(Elevation_Angle);
            W_Vel[rowcnt]=1;
            Z_Vel[rowcnt]=Single_Epoch_Obs.Doppler_GPS_Observation[prn]
                          +(P_Satellite.X-X_star)*sat_info.v_X/Rou_Vel[rowcnt]
                          +(P_Satellite.Y-Y_star)*sat_info.v_Y/Rou_Vel[rowcnt]
                          +(P_Satellite.Z-Z_star)*sat_info.v_Z/Rou_Vel[rowcnt]
                          +sat_info.Clkdelta_Dot*C_LIGHT;
            rowcnt++;
        }
        for (auto& [prn,sat_info] : BDS_PV)
        {
            //卫星位置
            Cartesian_XYZ_Coord P_Satellite=sat_info.Position;

            ENU ENU_Satellite= XYZ2ENU(P_Satellite,P_Receiver);
            double E=ENU_Satellite.E;
            double N=ENU_Satellite.N;
            double U=ENU_Satellite.U;

            double Elevation_Angle=std::atan2(U,std::sqrt((E*E+N*N)));
//            std::cout<<'C'<<prn<<' '<<Elevation_Angle*180/PI<<std::endl;
//            printf("U:%lf\nElevation_Angle:%lf\n",U,Elevation_Angle*180/PI);
            if(cnt!=1&&Elevation_Angle*180/PI<15)
            {
                obs_drop++;
                continue;
            }

            Rou_Vel[rowcnt]= std::sqrt(std::pow(P_Satellite.X-X_star,2)+std::pow(P_Satellite.Y-Y_star,2)+std::pow(P_Satellite.Z-Z_star,2));
            //为设计矩阵该行赋值
//            H_Vel[rowcnt*4+0]=(X_star-P_Satellite.X)/Rou_Vel[rowcnt];
//            H_Vel[rowcnt*4+1]=(Y_star-P_Satellite.Y)/Rou_Vel[rowcnt];
//            H_Vel[rowcnt*4+2]=(Z_star-P_Satellite.Z)/Rou_Vel[rowcnt];
            H_Vel[rowcnt*4+0]=(P_Satellite.X-X_star)/Rou_Vel[rowcnt];
            H_Vel[rowcnt*4+1]=(P_Satellite.Y-Y_star)/Rou_Vel[rowcnt];
            H_Vel[rowcnt*4+2]=(P_Satellite.Z-Z_star)/Rou_Vel[rowcnt];
            H_Vel[rowcnt*4+3]=1;
            //观测值
//            W[rowcnt]=Single_Epoch_Obs.Doppler_CNO_BDS_Observation[prn]/30;
//            W_Vel[rowcnt]=std::sin(Elevation_Angle);
            W_Vel[rowcnt]=1;
            Z_Vel[rowcnt]=Single_Epoch_Obs.Doppler_BDS_Observation[prn]
                          +(P_Satellite.X-X_star)*sat_info.v_X/Rou[rowcnt]
                          +(P_Satellite.Y-Y_star)*sat_info.v_Y/Rou[rowcnt]
                          +(P_Satellite.Z-Z_star)*sat_info.v_Z/Rou[rowcnt]
                          +sat_info.Clkdelta_Dot*C_LIGHT;

            rowcnt++;
        }

        if(l-obs_drop-4<0)
        {
            printf("观测数量不足\n");
            break;
        }

        Matrix H_matrix(l-obs_drop,4,H_Vel);

        //权矩阵，等权时实际上可以忽略
        Matrix P(l-obs_drop, W_Vel);

        //线性化后的 观测值向量
        Matrix Z_matrix(l-obs_drop,1,Z_Vel);
//        printf("Z_matrix:\n");
//        Z_matrix.print();
        //协因数矩阵
        Matrix Q=(H_matrix.transpose()*P*H_matrix).inverse();

        Vel_vector=Q*H_matrix.transpose()*P*Z_matrix;
        printf("Vel_vector:\n");
        Vel_vector.print();
        printf("%s\n",std::string(66, '-').c_str());

        Matrix V=H_matrix*Vel_vector-Z_matrix;
        Hat_Sigma_0=std::sqrt((V.transpose()*P*V).My_Matrix(0,0)/(l-obs_drop-4));
        printf("Hat_Sigma_0:%lf\n",Hat_Sigma_0);
        D_Vel=Q*Hat_Sigma_0*Hat_Sigma_0;
        D_Vel.print();

        Receiver_PV.v_X=Vel_vector.My_Matrix(0,0);
        Receiver_PV.v_Y=Vel_vector.My_Matrix(1,0);
        Receiver_PV.v_Z=Vel_vector.My_Matrix(2,0);
    }

    Novatel_Data.SPP_Result.push_back(Receiver_PV);
}