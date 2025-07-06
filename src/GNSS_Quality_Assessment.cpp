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
            if(prn[0]!='G')
            {
                continue;
            }
            if (Observation_Data.Time_Range.count(prn) == 0) {
                Observation_Data.Time_Range[prn].push_back({index, index});
            } else {
                auto &[l, r] = Observation_Data.Time_Range[prn].back();
                if (index == r + 1)
                {
                    if(Linear_Combine_Calc(Sat_Obs,prn))
                    {
                        Observation_Data.Rinex_Obs_Data[index].Satellites_Obs[prn]=Sat_Obs;
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




    double Tmp_Mean,Tmp_Sigma;
    for(auto &[prn,vRange]:Observation_Data.Time_Range)
    {
        if(prn[0]!='G')
        {
            continue;
        }
        int Epoch_index=0;
        for(auto &[start_index,end_index]:vRange)
        {
            static int offset=0;
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
                if(Tmp_Mean-MW_Mean[prn][Epoch_index-1]>4*std::sqrt(MW_Sigma[prn][Epoch_index-1]) )
                {
                    if(Epoch_index==end_index)
                    {
                        end_index--;
                        vRange.insert(vRange.begin()+offset,std::pair(Epoch_index,Epoch_index) );
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
                            vRange.insert(vRange.begin()+offset,std::pair(Epoch_index,Epoch_index) );
                            vRange.insert(vRange.begin()+offset+1,std::pair(Epoch_index+1,end_index) );
                            end_index=Epoch_index-1;
                        }
                        //若下一历元也超限 并且与本历元MW观测值差别较大 则认为也是粗差 差别不大则认为本历元发生了周跳
                        else
                        {
                            if(std::abs(Mean_next-Tmp_Mean)>1)
                            {
                                vRange.insert(vRange.begin()+offset,std::pair(Epoch_index,Epoch_index) );
                                vRange.insert(vRange.begin()+offset+1,std::pair(Epoch_index+1,end_index) );
                                end_index=Epoch_index-1;
                            }
                            else
                            {
                                vRange.insert(vRange.begin()+offset,std::pair(Epoch_index,end_index) );
                                end_index=Epoch_index-1;
                                Cycle_Slip_List[prn].push_back(Epoch_index);
                                printf("Epoch_index:%d\n",Epoch_index);
                            }
                        }

                    }
                }


                Epoch_index++;
            }

            offset++;
        }
    }




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
            }
            else
            {
                return false;
            }
            break;
        }
        case 'C':
        {
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































