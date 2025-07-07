//
// Created by asus on 2025/7/4.
//
#include "Data_Rinex.h"




Rinex_Observation_Completeness_Rate::Rinex_Observation_Completeness_Rate(Rinex304_Observation_Data &Observation_Data,
                                                                         double Freq_Of_Obs)
{
    double Duration_Of_Obs=Observation_Data.pHeader->Last_Obs_Time - Observation_Data.pHeader->First_Obs_Time;

    int A_j,B_j,C_j,D_j;//在观测时间段内实际观测历元总数以及理论历元总数
    double DI_Freq,DI_Sys;//单频点以及系统数据完整率

    int Theoretical_Cnt_Value=Duration_Of_Obs*Freq_Of_Obs;


    std::map<std::string,std::map<std::string,int> > cnt_map;

    for(auto &Epoch_Obs:Observation_Data.Rinex_Obs_Data)
    {
        for(auto& [prn,Sat_Obs]:Epoch_Obs.Satellites_Obs)
        {
            for(auto& [Obs_Type,Obs]:Sat_Obs.Single_Satellite_Obs)
            {
                if(Obs!=0)
                    cnt_map[prn][Obs_Type]++;
            }
        }
    }

    for(auto & [prn,Obs_info]:cnt_map)
    {
        for(auto & [Obs_Type,Actual_Cnt_Value]:Obs_info)
        {
            Freq_Observation_Completeness_Rate[prn[0]][Obs_Type[1]].first+=Actual_Cnt_Value;
            Freq_Observation_Completeness_Rate[prn[0]][Obs_Type[1]].second+=Theoretical_Cnt_Value;

            Sys_Observation_Completeness_Rate[prn[0]].first+=Actual_Cnt_Value;
            Sys_Observation_Completeness_Rate[prn[0]].second+=Theoretical_Cnt_Value;
        }
    }

    for(auto & [Sys_Flag,Obs_info]:Freq_Observation_Completeness_Rate)
    {
        for(auto & [Obs_Type,Cnt_Value]:Obs_info)
        {
            printf("Freq_Observation_Completeness_Rate Sys %c Freq %c:%lf\n",Sys_Flag,Obs_Type,
                   Cnt_Value.first*1.0/Cnt_Value.second);
        }
        printf("Sys _Observation_Completeness_Rate Sys %c:%lf\n",Sys_Flag,
               Sys_Observation_Completeness_Rate[Sys_Flag].first*1.0/Sys_Observation_Completeness_Rate[Sys_Flag].second);
    }

    return ;

}

Rinex_Observation_Cycle_Slip_Detection::Rinex_Observation_Cycle_Slip_Detection(Rinex304_Observation_Data &Observation_Data,
double Freq_Of_Obs)
{

    double Duration_Of_Obs=Observation_Data.pHeader->Last_Obs_Time - Observation_Data.pHeader->First_Obs_Time;
    int Num_Of_Epoch=Duration_Of_Obs*Freq_Of_Obs;
    //判断单颗卫星连续观测时间段
    int index=0;
    for(auto &Epoch_Obs: Observation_Data.Rinex_Obs_Data)
    {
        for (auto &[prn, Sat_Obs]: Epoch_Obs.Satellites_Obs)
        {
            if(prn[0]!='G'&&prn[0]!='C')
            {
                continue;
            }
            if (Observation_Data.Time_Range.count(prn) == 0)
            {
                if(Linear_Combine_Calc(Sat_Obs,prn))
                {
                    Observation_Data.Time_Range[prn].push_back({index, index});
                }
            }
            else {
                auto &[l, r] = Observation_Data.Time_Range[prn].back();
                if (index == r + 1)
                {
                    if(Linear_Combine_Calc(Sat_Obs,prn))
                    {
                        r = index;
                    }
                }
                else
                {
                    if (Linear_Combine_Calc(Sat_Obs, prn))
                    {
                        Observation_Data.Time_Range[prn].push_back({index, index});
                    }
                }
            }
        }
        index++;
    }

    MW_Cycle_Slip_Detection(Observation_Data, *this);
    GF_Cycle_Slip_Detection(Observation_Data, *this);
    Receiver_Clock_Slip_Detection(Observation_Data, *this);
    return ;
}



bool Linear_Combine_Calc(Rinex304_Single_Satellite_Observation_Data &Sat_Obs,std::string PRN)
{
    switch (PRN[0])
    {
        case 'G':
        {
            double Carrier_Phase_L1=0;
            double Carrier_Phase_L2=0;
            double Pseudorange_L1=0;
            double Pseudorange_L2=0;

            for(auto &[Obs_Type,Obs]:Sat_Obs.Single_Satellite_Obs)
            {
                if(Obs_Type[0]=='L')
                {
                    if(Obs_Type[1]=='1'&&Carrier_Phase_L1==0)
                    {
                        Carrier_Phase_L1=Obs*C_LIGHT/FREQ_GPS_L1;
                    }
                    if(Obs_Type[1]=='2'&&Carrier_Phase_L2==0)
                    {
                        Carrier_Phase_L2=Obs*C_LIGHT/FREQ_GPS_L2;
                    }
                }
                if(Obs_Type[0]=='C')
                {
                    if(Obs_Type[1]=='1'&&Pseudorange_L1==0)
                    {
                        Pseudorange_L1=Obs;
                    }
                    if(Obs_Type[1]=='2'&&Pseudorange_L2==0)
                    {
                        Pseudorange_L2=Obs;
                    }
                }
            }

            if(Carrier_Phase_L1!=0&&Carrier_Phase_L2!=0&&Pseudorange_L1!=0&&Pseudorange_L2!=0)
            {
                Sat_Obs.Melbourne_Wubbena=(FREQ_GPS_L1*Carrier_Phase_L1-FREQ_GPS_L2*Carrier_Phase_L2)/(FREQ_GPS_L1-FREQ_GPS_L2)
                                          -(FREQ_GPS_L1*Pseudorange_L1+FREQ_GPS_L2*Pseudorange_L2)/(FREQ_GPS_L1+FREQ_GPS_L2);
                Sat_Obs.Carrier_Phase_Geometry_Free=Carrier_Phase_L2-Carrier_Phase_L1;
                Sat_Obs.Pseudorange_Geometry_Free=Pseudorange_L2-Pseudorange_L1;

                Sat_Obs.Carrier_Phase1=Carrier_Phase_L1;
                Sat_Obs.Pseudorange1=Pseudorange_L1;
                Sat_Obs.Carrier_Phase2=Carrier_Phase_L2;
                Sat_Obs.Pseudorange2=Pseudorange_L2;
            }
            else
            {
                return false;
            }
            break;
        }
        case 'C':
        {
            double Carrier_Phase_B1=0;
            double Carrier_Phase_B2=0;
            double Carrier_Phase_B3=0;

            double Pseudorange_B1=0;
            double Pseudorange_B2=0;
            double Pseudorange_B3=0;

            for(auto &[Obs_Type,Obs]:Sat_Obs.Single_Satellite_Obs)
            {
                if(Obs_Type[0]=='L')
                {
                    if(Obs_Type[1]=='2'&&Carrier_Phase_B1==0)
                    {
                        Carrier_Phase_B1=Obs*C_LIGHT/FREQ_BDS_B1I;
                    }
                    if(Obs_Type[1]=='7'&&Carrier_Phase_B2==0)
                    {
                        Carrier_Phase_B2=Obs*C_LIGHT/FREQ_BDS_B2I;
                    }
                    if(Obs_Type[1]=='6'&&Carrier_Phase_B3==0)
                    {
                        Carrier_Phase_B3=Obs*C_LIGHT/FREQ_BDS_B3I;
                    }
                }
                if(Obs_Type[0]=='C')
                {
                    if(Obs_Type[1]=='2'&&Pseudorange_B1==0)
                    {
                        Pseudorange_B1=Obs;
                    }
                    if(Obs_Type[1]=='7'&&Pseudorange_B2==0)
                    {
                        Pseudorange_B2=Obs;
                    }
                    if(Obs_Type[1]=='6'&&Pseudorange_B3==0)
                    {
                        Pseudorange_B3=Obs;
                    }
                }
            }

            if(Carrier_Phase_B1!=0&&Carrier_Phase_B3!=0&&Pseudorange_B1!=0&&Pseudorange_B3!=0)
            {
                Sat_Obs.Melbourne_Wubbena=(FREQ_BDS_B1I*Carrier_Phase_B1-FREQ_BDS_B3I*Carrier_Phase_B3)/(FREQ_BDS_B1I-FREQ_BDS_B3I)
                                          -(FREQ_BDS_B1I*Pseudorange_B1+FREQ_BDS_B3I*Pseudorange_B3)/(FREQ_BDS_B1I+FREQ_BDS_B3I);
                Sat_Obs.Carrier_Phase_Geometry_Free=Carrier_Phase_B3-Carrier_Phase_B1;
                Sat_Obs.Pseudorange_Geometry_Free=Pseudorange_B3-Pseudorange_B1;

                Sat_Obs.Carrier_Phase1=Carrier_Phase_B1;
                Sat_Obs.Pseudorange1=Pseudorange_B1;
                Sat_Obs.Carrier_Phase2=Carrier_Phase_B3;
                Sat_Obs.Pseudorange2=Pseudorange_B3;
            }
            else
            {
                return false;
            }
            break;
        }
        case 'R':
        {
            break;
        }
        case 'E':
        {
            break;
        }
        default:
            break;
    }
    return true;
}

void MW_Cycle_Slip_Detection(Rinex304_Observation_Data &Observation_Data,Rinex_Observation_Cycle_Slip_Detection &Cycle_Slip_Detection)
{
    double Tmp_Mean,Tmp_Sigma;
    for(auto &[prn,vRange]:Observation_Data.Time_Range) {
        if (prn[0] != 'G' && prn[0] != 'C') {
            continue;
        }
        int Epoch_index = 0;
        int offset = 0;
        for (size_t i = 0; i < vRange.size(); ++i) {
            auto &[start_index, end_index] = vRange[i];

            if (start_index == end_index) {
                Epoch_index++;
                offset++;
                continue;
            }
            while (Epoch_index <= end_index) {
                double MW_Value = Observation_Data.Rinex_Obs_Data[Epoch_index].Satellites_Obs[prn].Melbourne_Wubbena;

                if (Epoch_index == start_index) {
                    Tmp_Mean = MW_Value;
                    Tmp_Sigma = 0;
                    Cycle_Slip_Detection.MW_Mean[prn][Epoch_index] = Tmp_Mean;
                    Cycle_Slip_Detection.MW_Sigma[prn][Epoch_index] = Tmp_Sigma;
                }
                if (Epoch_index > start_index && Epoch_index <= end_index) {
                    Tmp_Mean = MW_Value / (Epoch_index + 1) +
                               Cycle_Slip_Detection.MW_Mean[prn][Epoch_index - 1] * Epoch_index / (Epoch_index + 1);
                    Tmp_Sigma = Cycle_Slip_Detection.MW_Sigma[prn][Epoch_index - 1] * Epoch_index / (Epoch_index + 1)
                                + (MW_Value - Cycle_Slip_Detection.MW_Mean[prn][Epoch_index - 1]) *
                                  (MW_Value - Cycle_Slip_Detection.MW_Mean[prn][Epoch_index - 1]) / (Epoch_index + 1);
                    Cycle_Slip_Detection.MW_Mean[prn][Epoch_index] = Tmp_Mean;
                    Cycle_Slip_Detection.MW_Sigma[prn][Epoch_index] = Tmp_Sigma;
                }
                //超限
                if (Epoch_index > start_index + 1 && Tmp_Mean - Cycle_Slip_Detection.MW_Mean[prn][Epoch_index - 1] >
                4 *std::sqrt(Cycle_Slip_Detection.MW_Sigma[prn][Epoch_index -1])) {
                    if (Epoch_index == end_index)
                    {
                        vRange[i].second = Epoch_index - 1;
                        vRange.insert(vRange.begin() + offset + 1, std::pair(Epoch_index, Epoch_index));
                        Cycle_Slip_Detection.Gross_Error_List[prn].push_back(Epoch_index);
                        printf("1  Gross_Error in Epoch_index:%d\n", Epoch_index);
                        continue;
                    } else {
                        double MW_Value_next = Observation_Data.Rinex_Obs_Data[Epoch_index +
                                                                               1].Satellites_Obs[prn].Melbourne_Wubbena;
                        double Mean_next =
                                MW_Value_next / (Epoch_index + 2) + Tmp_Mean * (Epoch_index + 1) / (Epoch_index + 2);
                        double Sigma_next = Tmp_Sigma * (Epoch_index + 1) / (Epoch_index + 2)
                                            +
                                            (MW_Value_next - Tmp_Mean) * (MW_Value_next - Tmp_Mean) / (Epoch_index + 2);
                        //若下一历元未超限 则本历元为粗差
                        if (Mean_next - Tmp_Mean < 4 * std::sqrt(Tmp_Sigma)) {
                            vRange.insert(vRange.begin() + offset + 1, std::pair(Epoch_index, Epoch_index));
                            vRange.insert(vRange.begin() + offset + 2, std::pair(Epoch_index + 1, end_index));
                            vRange[i].second = Epoch_index - 1;
                            Cycle_Slip_Detection.Gross_Error_List[prn].push_back(Epoch_index);
                            printf("2  Gross_Error in Epoch_index:%d prn:%s\n", Epoch_index,prn.c_str());
                            continue;
                        }
                            //若下一历元也超限 并且与本历元MW观测值差别较大 则认为也是粗差 差别不大则认为本历元发生了周跳
                        else {
                            if (std::abs(Mean_next - Tmp_Mean) > 1) {
                                vRange.insert(vRange.begin() + offset + 1, std::pair(Epoch_index, Epoch_index));
                                vRange.insert(vRange.begin() + offset + 2, std::pair(Epoch_index + 1, end_index));
                                vRange[i].second = Epoch_index - 1;
                                Cycle_Slip_Detection.Gross_Error_List[prn].push_back(Epoch_index);
                                printf("3  Gross_Error in Epoch_index:%d\n", Epoch_index);
                                continue;
                            } else {
                                vRange.insert(vRange.begin() + offset + 1, std::pair(Epoch_index, end_index));
                                vRange[i].second = Epoch_index - 1;
                                Cycle_Slip_Detection.Cycle_Slip_List[prn].push_back(Epoch_index);
                                printf("Epoch_index:%d\n", Epoch_index);
                                continue;
                            }
                        }

                    }
                }
                Epoch_index++;
            }

            offset++;
        }
    }
    return;
}

void GF_Cycle_Slip_Detection(Rinex304_Observation_Data &Observation_Data,Rinex_Observation_Cycle_Slip_Detection &Cycle_Slip_Detection)
{
    for(auto &[prn,vRange]:Observation_Data.Time_Range)
    {
        if (prn[0] != 'G' && prn[0] != 'C') {
            continue;
        }
        for (size_t i = 0; i < vRange.size(); ++i)
        {
            auto &[start_index, end_index] = vRange[i];
            //连续观测历元过少不易拟合
            if(end_index-start_index<100)
            {
                continue;
            }
            //存在周跳的卫星不参与该补充探测
            if(Cycle_Slip_Detection.Cycle_Slip_List.count(prn)!=0)
            {
                continue;
            }

            //拟合该卫星的伪距GF组合观测值
            std::vector<double> Carrier_Phase_GF;
            std::vector<double> Pseudorange_GF;
            std::vector<double> Epoch_index;

            std::vector<double> Theoretical_Pseudorange_GF;

            int index=start_index;
            while(index<=end_index)
            {
                Carrier_Phase_GF.push_back(Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Carrier_Phase_Geometry_Free);
                Pseudorange_GF.push_back(Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Pseudorange_Geometry_Free);
                Epoch_index.push_back(index);
                index++;
            }

            int degree=6;
            if(int((end_index-start_index)/100+0.5)+1<degree )
            {
                degree=int((end_index-start_index)/100)+1;
            }
            auto Fit_Result=Poly_Fit(Epoch_index,Pseudorange_GF,degree);
            for(auto &x:Epoch_index)
            {
                Theoretical_Pseudorange_GF.push_back(Poly_Value(Fit_Result,x));
            }

            //遍历除去首尾历元外的历元
            for (size_t j = 1; j < Carrier_Phase_GF.size()-1; ++j)
            {
                double L_GF1=Carrier_Phase_GF[j-1];
                double L_GF2=Carrier_Phase_GF[j];
                double L_GF3=Carrier_Phase_GF[j+1];

                double Q_GF1=Theoretical_Pseudorange_GF[j-1];
                double Q_GF2=Theoretical_Pseudorange_GF[j];
                double Q_GF3=Theoretical_Pseudorange_GF[j+1];

                switch (prn[0])
                {
                    case 'G':
                    {
                        if( std::abs(L_GF2-L_GF1-Q_GF2+Q_GF1)>6*(C_LIGHT/FREQ_GPS_L2-C_LIGHT/FREQ_GPS_L1)
                        &&  std::abs(L_GF3-L_GF2-Q_GF3+Q_GF2)>(C_LIGHT/FREQ_GPS_L2-C_LIGHT/FREQ_GPS_L1) )
                        {
                            vRange.insert(vRange.begin() + i + 1, std::pair(Epoch_index[j], end_index));
                            vRange[i].second = Epoch_index[j] - 1;
                            Cycle_Slip_Detection.Cycle_Slip_List[prn].push_back(Epoch_index[j]);
                            printf("Cycle_Slip in Epoch_index:%d\n", Epoch_index[j]);
                        }
                        break;
                    }
                    case 'C':
                    {
                        if( std::abs(L_GF2-L_GF1-Q_GF2+Q_GF1)>6*(C_LIGHT/FREQ_BDS_B3I-C_LIGHT/FREQ_BDS_B1I)
                            &&  std::abs(L_GF3-L_GF2-Q_GF3+Q_GF2)>(C_LIGHT/FREQ_BDS_B3I-C_LIGHT/FREQ_BDS_B1I) )
                        {
                            vRange.insert(vRange.begin() + i + 1, std::pair(Epoch_index[j], end_index));
                            vRange[i].second = Epoch_index[j] - 1;
                            Cycle_Slip_Detection.Cycle_Slip_List[prn].push_back(Epoch_index[j]);
                            printf("Cycle_Slip in Epoch_index:%d\n", Epoch_index[j]);
                        }
                        break;
                    }
                    case 'R':
                    {
                        break;
                    }
                    case 'E':
                    {
                        break;
                    }
                    default:
                        break;
                }

            }


        }

    }

    return;
}

// 多项式拟合函数
Eigen::VectorXd Poly_Fit(const std::vector<double>& x, const std::vector<double>& y, int degree) {
    if (x.size() != y.size()) {
        throw std::runtime_error("x and y must have the same size.");
    }

    const int N = x.size();
    Eigen::MatrixXd A(N, degree + 1);
    Eigen::VectorXd b(N);

    // 构建范德蒙矩阵
    for (int i = 0; i < N; ++i) {
        double xi = 1.0;
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = xi;
            xi *= x[i];
        }
        b(i) = y[i];
    }

    // 最小二乘解
    Eigen::VectorXd coeff = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    return coeff;
}

// 用拟合结果计算函数值
double Poly_Value(const Eigen::VectorXd& coeffs, double x) {
    double result = 0.0;
    double xi = 1.0;
    for (int i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * xi;
        xi *= x;
    }
    return result;
}

void Receiver_Clock_Slip_Detection(Rinex304_Observation_Data &Observation_Data,Rinex_Observation_Cycle_Slip_Detection &Cycle_Slip_Detection)
{
    //观测噪声经验值，默认大小为4，单位m
    double Xi=4;

    for(auto &[prn,vRange]:Observation_Data.Time_Range) {
        if (prn[0] != 'G' && prn[0] != 'C') {
            continue;
        }
        //存在周跳的卫星不参与钟跳检测
        if(Cycle_Slip_Detection.Cycle_Slip_List.count(prn)!=0)
        {
            continue;
        }
        for(auto &[start_index,end_index]:vRange)
        {
            int index=start_index;
            while(index<end_index)
            {
                double Rou_1=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Pseudorange1;
                double Rou_2=Observation_Data.Rinex_Obs_Data[index+1].Satellites_Obs[prn].Pseudorange1;
                double Phi_1=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Carrier_Phase1;
                double Phi_2=Observation_Data.Rinex_Obs_Data[index+1].Satellites_Obs[prn].Carrier_Phase1;

                int Tmp=std::abs(Rou_2-Rou_1-Phi_2+Phi_1);
                //存在
                if(C_LIGHT*1e-7-3*Xi<Tmp && Tmp<C_LIGHT*1e-5-3*Xi)
                {
                    Cycle_Slip_Detection.Receiver_Clock_Slip[index][prn]=1;
                }else if(Tmp>C_LIGHT*1e-5-3*Xi)
                {
                    Cycle_Slip_Detection.Receiver_Clock_Slip[index][prn]=2;
                } else
                {
                    Cycle_Slip_Detection.Receiver_Clock_Slip[index][prn]=0;
                }
                index++;
            }
        }
    }

    return;
}

Rinex_Observation_MultiPath_Detection::Rinex_Observation_MultiPath_Detection(Rinex304_Observation_Data &Observation_Data,double Freq_Of_Obs)
{
    double Duration_Of_Obs=Observation_Data.pHeader->Last_Obs_Time - Observation_Data.pHeader->First_Obs_Time;
    int Num_Of_Epoch=Duration_Of_Obs*Freq_Of_Obs;
    //判断单颗卫星连续观测时间段
    for(auto &[prn,vRange]:Observation_Data.Time_Range)
    {
        if (prn[0] != 'G' && prn[0] != 'C') {
            continue;
        }
        for (auto &[start_index, end_index] : vRange)
        {
            if(end_index-start_index<100)
            {
                continue;
            }
            int index=start_index;
            MultiPath_Freq1_PrefixSum[prn][index]=0;
            MultiPath_Freq2_PrefixSum[prn][index]=0;
            while(index<=end_index)
            {
                switch (prn[0])
                {
                    case 'G':
                    {
                        double MP_L1,MP_L2;
                        double Rou_1=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Pseudorange1;
                        double Rou_2=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Pseudorange2;
                        double Phi_1=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Carrier_Phase1;
                        double Phi_2=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Carrier_Phase2;

                        MP_L1=Rou_1
                              -(FREQ_GPS_L1*FREQ_GPS_L1+FREQ_GPS_L2*FREQ_GPS_L2)/(FREQ_GPS_L1*FREQ_GPS_L1-FREQ_GPS_L2*FREQ_GPS_L2)*Phi_1
                              +(2*FREQ_GPS_L2*FREQ_GPS_L2)/(FREQ_GPS_L1*FREQ_GPS_L1-FREQ_GPS_L2*FREQ_GPS_L2)*Phi_2;
                        MP_L2=Rou_2
                              -(2*FREQ_GPS_L1*FREQ_GPS_L1)/(FREQ_GPS_L1*FREQ_GPS_L1-FREQ_GPS_L2*FREQ_GPS_L2)*Phi_1
                              +(FREQ_GPS_L1*FREQ_GPS_L1+FREQ_GPS_L2*FREQ_GPS_L2)/(FREQ_GPS_L1*FREQ_GPS_L1-FREQ_GPS_L2*FREQ_GPS_L2)*Phi_2;

                        MultiPath_Freq1[prn][index]=MP_L1;
                        MultiPath_Freq2[prn][index]=MP_L2;
                        MultiPath_Freq1_PrefixSum[prn][index+1]=MultiPath_Freq1_PrefixSum[prn][index]+MP_L1;
                        MultiPath_Freq2_PrefixSum[prn][index+1]=MultiPath_Freq2_PrefixSum[prn][index]+MP_L2;

                        break;
                    }
                    case 'C':
                    {
                        double MP_L1,MP_L2;
                        double Rou_1=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Pseudorange1;
                        double Rou_2=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Pseudorange2;
                        double Phi_1=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Carrier_Phase1;
                        double Phi_2=Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn].Carrier_Phase2;

                        MP_L1=Rou_1
                              -(FREQ_BDS_B1I*FREQ_BDS_B1I+FREQ_BDS_B3I*FREQ_BDS_B3I)/(FREQ_BDS_B1I*FREQ_BDS_B1I-FREQ_BDS_B3I*FREQ_BDS_B3I)*Phi_1
                              +(2*FREQ_BDS_B3I*FREQ_BDS_B3I)/(FREQ_BDS_B1I*FREQ_BDS_B1I-FREQ_BDS_B3I*FREQ_BDS_B3I)*Phi_2;
                        MP_L2=Rou_2
                              -(2*FREQ_BDS_B1I*FREQ_BDS_B1I)/(FREQ_BDS_B1I*FREQ_BDS_B1I-FREQ_BDS_B3I*FREQ_BDS_B3I)*Phi_1
                              +(FREQ_BDS_B1I*FREQ_BDS_B1I+FREQ_BDS_B3I*FREQ_BDS_B3I)/(FREQ_BDS_B1I*FREQ_BDS_B1I-FREQ_BDS_B3I*FREQ_BDS_B3I)*Phi_2;

                        MultiPath_Freq1[prn][index]=MP_L1;
                        MultiPath_Freq2[prn][index]=MP_L2;
                        MultiPath_Freq1_PrefixSum[prn][index+1]=MultiPath_Freq1_PrefixSum[prn][index]+MP_L1;
                        MultiPath_Freq2_PrefixSum[prn][index+1]=MultiPath_Freq2_PrefixSum[prn][index]+MP_L2;

                        break;
                    }
                    case 'R':
                    {
                        break;
                    }
                    case 'E':
                    {
                        break;
                    }
                    default:
                        break;
                }
                index++;
            }

            double Sum_MP_L1,Sum_MP_L2,Tmp_L1,Tmp_L2;
            Sum_MP_L1=MultiPath_Freq1_PrefixSum[prn][end_index+1]-MultiPath_Freq1_PrefixSum[prn][start_index];
            Sum_MP_L2=MultiPath_Freq2_PrefixSum[prn][end_index+1]-MultiPath_Freq2_PrefixSum[prn][start_index];
            Tmp_L1=0;
            Tmp_L2=0;

            index=0;
            while(index<=end_index)
            {
                switch (prn[0])
                {
                    case 'G':
                    {
                        double MP_L1,MP_L2;
                        MP_L1=MultiPath_Freq1[prn][index];
                        MP_L2=MultiPath_Freq2[prn][index];
                        Tmp_L1+=(MP_L1-Sum_MP_L1/(end_index-start_index+1))*(MP_L1-Sum_MP_L1/(end_index-start_index+1));
                        Tmp_L2+=(MP_L2-Sum_MP_L2/(end_index-start_index+1))*(MP_L2-Sum_MP_L2/(end_index-start_index+1));
                        break;
                    }
                    case 'C':
                    {
                        double MP_L1,MP_L2;
                        MP_L1=MultiPath_Freq1[prn][index];
                        MP_L2=MultiPath_Freq2[prn][index];
                        Tmp_L1+=(MP_L1-Sum_MP_L1/(end_index-start_index+1))*(MP_L1-Sum_MP_L1/(end_index-start_index+1));
                        Tmp_L2+=(MP_L2-Sum_MP_L2/(end_index-start_index+1))*(MP_L2-Sum_MP_L2/(end_index-start_index+1));
                        break;
                    }
                    case 'R':
                    {
                        break;
                    }
                    case 'E':
                    {
                        break;
                    }
                    default:
                        break;
                }
                index++;
            }

            Tmp_L1/=(end_index-start_index);
            Tmp_L2/=(end_index-start_index);


            MultiPath_Freq1_Assessment[prn].push_back(std::sqrt(Tmp_L1));
            MultiPath_Freq2_Assessment[prn].push_back(std::sqrt(Tmp_L2));
        }
    }




return ;

}


//已弃用 更改为了函数 MW_Cycle_Slip_Detection
/*
    double Tmp_Mean,Tmp_Sigma;
    for(auto &[prn,vRange]:Observation_Data.Time_Range)
    {
        if(prn[0]!='G'&&prn[0]!='C')
        {
            continue;
        }
        int Epoch_index=0;
        int offset=0;
        for (size_t i = 0; i < vRange.size(); ++i)
        {
            auto &[start_index, end_index] = vRange[i];

            if(start_index==end_index)
            {
                Epoch_index++;
                offset++;
                continue;
            }
            while(Epoch_index<=end_index)
            {
                double MW_Value=Observation_Data.Rinex_Obs_Data[Epoch_index].Satellites_Obs[prn].Melbourne_Wubbena;

                if(Epoch_index==start_index)
                {
                    Tmp_Mean=MW_Value;
                    Tmp_Sigma=0;
                    MW_Mean[prn][Epoch_index]=Tmp_Mean;
                    MW_Sigma[prn][Epoch_index]=Tmp_Sigma;
                }
                if(Epoch_index>start_index&&Epoch_index<=end_index)
                {
                    Tmp_Mean=MW_Value/(Epoch_index+1)+MW_Mean[prn][Epoch_index-1]*Epoch_index/(Epoch_index+1);
                    Tmp_Sigma=MW_Sigma[prn][Epoch_index-1]*Epoch_index/(Epoch_index+1)
                              +(MW_Value-MW_Mean[prn][Epoch_index-1])*(MW_Value-MW_Mean[prn][Epoch_index-1])/(Epoch_index+1);
                    MW_Mean[prn][Epoch_index]=Tmp_Mean;
                    MW_Sigma[prn][Epoch_index]=Tmp_Sigma;
                }
                //超限
                if(Epoch_index>start_index+1&&Tmp_Mean-MW_Mean[prn][Epoch_index-1]>4*std::sqrt(MW_Sigma[prn][Epoch_index-1]) )
                {
                    if(Epoch_index==end_index)
                    {
                        vRange[i].second=Epoch_index-1;
                        vRange.insert(vRange.begin()+offset+1,std::pair(Epoch_index,Epoch_index) );
                        Gross_Error_List[prn].push_back(Epoch_index);
                        printf("Gross_Error in Epoch_index:%d\n",Epoch_index);
                        continue;
                    }
                    else
                    {
                        double MW_Value_next = Observation_Data.Rinex_Obs_Data[Epoch_index+1].Satellites_Obs[prn].Melbourne_Wubbena;
                        double Mean_next = MW_Value_next/(Epoch_index+2)+Tmp_Mean*(Epoch_index+1)/(Epoch_index+2);
                        double Sigma_next = Tmp_Sigma*(Epoch_index+1)/(Epoch_index+2)
                                            +(MW_Value_next-Tmp_Mean)*(MW_Value_next-Tmp_Mean)/(Epoch_index+2);
                        //若下一历元未超限 则本历元为粗差
                        if(Mean_next-Tmp_Mean<4*std::sqrt(Tmp_Sigma) )
                        {
                            vRange.insert(vRange.begin()+offset+1,std::pair(Epoch_index,Epoch_index) );
                            vRange.insert(vRange.begin()+offset+2,std::pair(Epoch_index+1,end_index) );
                            vRange[i].second=Epoch_index-1;
                            Gross_Error_List[prn].push_back(Epoch_index);
                            printf("Gross_Error in Epoch_index:%d\n",Epoch_index);
                            continue;
                        }
                            //若下一历元也超限 并且与本历元MW观测值差别较大 则认为也是粗差 差别不大则认为本历元发生了周跳
                        else
                        {
                            if(std::abs(Mean_next-Tmp_Mean)>1)
                            {
                                vRange.insert(vRange.begin()+offset+1,std::pair(Epoch_index,Epoch_index) );
                                vRange.insert(vRange.begin()+offset+2,std::pair(Epoch_index+1,end_index) );
                                vRange[i].second=Epoch_index-1;
                                Gross_Error_List[prn].push_back(Epoch_index);
                                printf("Gross_Error in Epoch_index:%d\n",Epoch_index);
                                continue;
                            }
                            else
                            {
                                vRange.insert(vRange.begin()+offset+1,std::pair(Epoch_index,end_index) );
                                vRange[i].second=Epoch_index-1;
                                Cycle_Slip_List[prn].push_back(Epoch_index);
                                printf("Epoch_index:%d\n",Epoch_index);
                                continue;
                            }
                        }

                    }
                }
                Epoch_index++;
            }

            offset++;
        }
    }
*/




















