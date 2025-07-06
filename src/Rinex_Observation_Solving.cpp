//
// Created by asus on 2025/7/3.
//
#include "Data_Rinex.h"


double Read_Double_Data(const std::string& line, int offset,int length)
{
    std::string field;
    if (line.size() > offset) {
        field = line.substr(offset, length);
    } else {
        return 0.0;
    }
    if(isAllSpaces(field))
        return 0.0;
    else
        return std::stod(field);
}
int Read_Int_Data(const std::string& line, int offset,int length)
{
    std::string field;
    if (line.size() > offset) {
        field = line.substr(offset, length);
    } else {
        return 0;
    }
    if(isAllSpaces(field))
        return 0;
    else
        return std::stoi(field);
}

Rinex304_Observation_Head::Rinex304_Observation_Head(std::ifstream& file)
{
    std::string line;
    while (std::getline(file, line)) {
        // RINEX 版本和类型
        if (line.find("RINEX VERSION / TYPE") != std::string::npos)
        {
            version = Read_Double_Data(line,0,9);  // 提取版本号
            // 拷贝类型字段
            std::strncpy(type, line.substr(20, 20).c_str(), 20);  // 拷贝类型字段
        }
        //APPROX POSITION XYZ 位置
        if (line.find("APPROX POSITION XYZ") != std::string::npos)
        {
            Approximate_Position.X = Read_Double_Data(line,0,14);
            Approximate_Position.Y = Read_Double_Data(line,14,14);
            Approximate_Position.Z = Read_Double_Data(line,28,14);
        }

        //TIME OF FIRST OBS
        if (line.find("TIME OF FIRST OBS") != std::string::npos)
        {
            int year=Read_Int_Data(line,0,6);
            int month=Read_Int_Data(line,6,6);
            int day=Read_Int_Data(line,12,6);
            int hour=Read_Int_Data(line,18,6);
            int min=Read_Int_Data(line,24,6);
            double sec= Read_Double_Data(line,30,13);
            First_Obs_Time=GPST(year,month,day,hour,min,sec);
        }

        //TIME OF LAST OBS
        if (line.find("TIME OF LAST OBS") != std::string::npos)
        {
            int year=Read_Int_Data(line,0,6);
            int month=Read_Int_Data(line,6,6);
            int day=Read_Int_Data(line,12,6);
            int hour=Read_Int_Data(line,18,6);
            int min=Read_Int_Data(line,24,6);
            double sec= Read_Double_Data(line,30,13);
            Last_Obs_Time=GPST(year,month,day,hour,min,sec);
        }

        //SYS / # / OBS TYPES 观测值类型
        if (line.find("SYS / # / OBS TYPES") != std::string::npos)
        {
            std::string Sys_Flag=line.substr(0, 1);
            int Num_Of_Type=Read_Int_Data(line,3,3);
            switch (Sys_Flag[0])
            {
                case 'G':
                {
                    //GPS系统
                    int line_num=(Num_Of_Type+12)/13;
                    for(int j=1;j<line_num;j++)
                    {
                        for (int i=0;i<13;i++)
                        {
                            Sys_Obs_Type['G'].push_back(line.substr(6+4*i+1, 3));
                        }
                        std::getline(file, line);
                    }
                    for (int i = 0; i < Num_Of_Type%13; i++) {
                        Sys_Obs_Type['G'].push_back(line.substr(6 + 4 * i + 1, 3));
                    }
                    break;
                }
                case 'C':
                {
                    //北斗系统
                    int line_num=(Num_Of_Type+12)/13;
                    for(int j=1;j<line_num;j++)
                    {
                        for (int i=0;i<13;i++)
                        {
                            Sys_Obs_Type['C'].push_back(line.substr(6+4*i+1, 3));
                        }
                        std::getline(file, line);
                    }
                    for (int i = 0; i < Num_Of_Type%13; i++) {
                        Sys_Obs_Type['C'].push_back(line.substr(6 + 4 * i + 1, 3));
                    }
                    break;
                }
                case 'R':
                {
                    //GLONASS系统
                    int line_num=(Num_Of_Type+12)/13;
                    for(int j=1;j<line_num;j++)
                    {
                        for (int i=0;i<13;i++)
                        {
                            Sys_Obs_Type['R'].push_back(line.substr(6+4*i+1, 3));
                        }
                        std::getline(file, line);
                    }
                    for (int i = 0; i < Num_Of_Type%13; i++) {
                        Sys_Obs_Type['R'].push_back(line.substr(6 + 4 * i + 1, 3));
                    }
                    break;
                }
                case 'E':
                {
                    //GALILEO系统
                    int line_num=(Num_Of_Type+12)/13;
                    for(int j=1;j<line_num;j++)
                    {
                        for (int i=0;i<13;i++)
                        {
                            Sys_Obs_Type['E'].push_back(line.substr(6+4*i+1, 3));
                        }
                        std::getline(file, line);
                    }
                    for (int i = 0; i < Num_Of_Type%13; i++) {
                        Sys_Obs_Type['E'].push_back(line.substr(6 + 4 * i + 1, 3));
                    }
                    break;
                }
                default:
                    break;
            }
        }

        //频点观测历元
        if (line.find("PRN / # OF OBS") != std::string::npos)
        {

        }
        // 文件头结束
        if (line.find("END OF HEADER") != std::string::npos)
        {
            break;
        }
    }
}

Rinex304_Single_Epoch_Observation_Data::Rinex304_Single_Epoch_Observation_Data(std::ifstream& file, Rinex304_Observation_Head* pHeader)
{
    std::string line;
    std::getline(file, line);

    Epoch_Time.year=Read_Int_Data(line,2,4);
    Epoch_Time.month=Read_Int_Data(line,7,2);
    Epoch_Time.day=Read_Int_Data(line,10,2);
    Epoch_Time.hour=Read_Int_Data(line,13,2);
    Epoch_Time.min=Read_Int_Data(line,16,2);
    Epoch_Time.sec= Read_Double_Data(line,18,11);

    Epoch_Flag=Read_Int_Data(line,31,1);

    Num_Of_Satellites=Read_Int_Data(line,32,3);

    for (int i = 0; i < Num_Of_Satellites; i++)
    {
        std::getline(file, line);
        std::string PRN=line.substr(0,3);
        switch (PRN[0])
        {
            case 'G':
            {
                Rinex304_Single_Satellite_Observation_Data Tmp_Obs_Data;
                int offset=3;
                for (auto &Obs_Type:(*pHeader).Sys_Obs_Type['G'])
                {
                    double Obs= Read_Double_Data(line,offset,14);
                    offset+=16;
//                    offset+=14+1;
//                    int Signal_Strength= Read_Int_Data(line,offset,1);
//                    offset+=1;
                    Tmp_Obs_Data.Single_Satellite_Obs[Obs_Type]=Obs;
                }



//                if(Tmp_Obs_Data.Pseudorange_1&&Tmp_Obs_Data.Pseudorange_2)
//                    Tmp_Obs_Data.Pseudorange_Geometry_Free=Tmp_Obs_Data.Pseudorange_1-Tmp_Obs_Data.Pseudorange_2;
//                if(Tmp_Obs_Data.Carrier_Phase_1&&Tmp_Obs_Data.Carrier_Phase_2)
//                    Tmp_Obs_Data.Carrier_Phase_Geometry_Free=C_LIGHT*(Tmp_Obs_Data.Carrier_Phase_1/FREQ_GPS_L1-Tmp_Obs_Data.Carrier_Phase_2/FREQ_GPS_L2);
//                if(Tmp_Obs_Data.Carrier_Phase_1&&Tmp_Obs_Data.Carrier_Phase_2&&Tmp_Obs_Data.Pseudorange_1&&Tmp_Obs_Data.Pseudorange_2)
//                    Tmp_Obs_Data.Melbourne_Wubbena=C_LIGHT/(FREQ_GPS_L1-FREQ_GPS_L2)*(Tmp_Obs_Data.Carrier_Phase_1-Tmp_Obs_Data.Carrier_Phase_2)
//                                                   -(FREQ_GPS_L1*Tmp_Obs_Data.Pseudorange_1+FREQ_GPS_L2*Tmp_Obs_Data.Pseudorange_2)/(FREQ_GPS_L1+FREQ_GPS_L2);;

                Satellites_Obs[PRN]=Tmp_Obs_Data;
                break;
            }
            case 'C':
            {
                Rinex304_Single_Satellite_Observation_Data Tmp_Obs_Data;
                int offset=3;
                for (auto &Obs_Type:(*pHeader).Sys_Obs_Type['C'])
                {
                    double Obs= Read_Double_Data(line,offset,14);
                    offset+=16;
//                    offset+=14+1;
//                    int Signal_Strength= Read_Int_Data(line,offset,1);
//                    offset+=1;
                    Tmp_Obs_Data.Single_Satellite_Obs[Obs_Type]=Obs;
                }

//                Tmp_Obs_Data.Pseudorange_Geometry_Free=Tmp_Obs_Data.Pseudorange_1-Tmp_Obs_Data.Pseudorange_2;
//                Tmp_Obs_Data.Carrier_Phase_Geometry_Free=C_LIGHT*(Tmp_Obs_Data.Carrier_Phase_1/FREQ_BDS_B1I-Tmp_Obs_Data.Carrier_Phase_2/FREQ_BDS_B3I);
//                Tmp_Obs_Data.Melbourne_Wubbena=C_LIGHT/(FREQ_BDS_B1I-FREQ_BDS_B3I)*(Tmp_Obs_Data.Carrier_Phase_1-Tmp_Obs_Data.Carrier_Phase_2)
//                                               -(FREQ_BDS_B1I*Tmp_Obs_Data.Pseudorange_1+FREQ_BDS_B3I*Tmp_Obs_Data.Pseudorange_2)/(FREQ_BDS_B1I+FREQ_BDS_B3I);;

                Satellites_Obs[PRN]=Tmp_Obs_Data;
                break;
            }
            case 'R':
            {
                Rinex304_Single_Satellite_Observation_Data Tmp_Obs_Data;
                int offset=3;
                for (auto &Obs_Type:(*pHeader).Sys_Obs_Type['R'])
                {
                    double Obs= Read_Double_Data(line,offset,14);
                    offset+=16;
//                    offset+=14+1;
//                    int Signal_Strength= Read_Int_Data(line,offset,1);
//                    offset+=1;
                    Tmp_Obs_Data.Single_Satellite_Obs[Obs_Type]=Obs;
                }

//                Tmp_Obs_Data.Pseudorange_Geometry_Free=Tmp_Obs_Data.Pseudorange_1-Tmp_Obs_Data.Pseudorange_2;
//                Tmp_Obs_Data.Carrier_Phase_Geometry_Free=C_LIGHT*(Tmp_Obs_Data.Carrier_Phase_1/FREQ_BDS_B1I-Tmp_Obs_Data.Carrier_Phase_2/FREQ_BDS_B3I);
//                Tmp_Obs_Data.Melbourne_Wubbena=C_LIGHT/(FREQ_BDS_B1I-FREQ_BDS_B3I)*(Tmp_Obs_Data.Carrier_Phase_1-Tmp_Obs_Data.Carrier_Phase_2)
//                                               -(FREQ_BDS_B1I*Tmp_Obs_Data.Pseudorange_1+FREQ_BDS_B3I*Tmp_Obs_Data.Pseudorange_2)/(FREQ_BDS_B1I+FREQ_BDS_B3I);;

                Satellites_Obs[PRN]=Tmp_Obs_Data;
                break;
            }
            case 'E':
            {
                Rinex304_Single_Satellite_Observation_Data Tmp_Obs_Data;
                int offset=3;
                for (auto &Obs_Type:(*pHeader).Sys_Obs_Type['E'])
                {
                    double Obs= Read_Double_Data(line,offset,14);
                    offset+=16;
//                    offset+=14+1;
//                    int Signal_Strength= Read_Int_Data(line,offset,1);
//                    offset+=1;
                    Tmp_Obs_Data.Single_Satellite_Obs[Obs_Type]=Obs;
                }

//                Tmp_Obs_Data.Pseudorange_Geometry_Free=Tmp_Obs_Data.Pseudorange_1-Tmp_Obs_Data.Pseudorange_2;
//                Tmp_Obs_Data.Carrier_Phase_Geometry_Free=C_LIGHT*(Tmp_Obs_Data.Carrier_Phase_1/FREQ_BDS_B1I-Tmp_Obs_Data.Carrier_Phase_2/FREQ_BDS_B3I);
//                Tmp_Obs_Data.Melbourne_Wubbena=C_LIGHT/(FREQ_BDS_B1I-FREQ_BDS_B3I)*(Tmp_Obs_Data.Carrier_Phase_1-Tmp_Obs_Data.Carrier_Phase_2)
//                                               -(FREQ_BDS_B1I*Tmp_Obs_Data.Pseudorange_1+FREQ_BDS_B3I*Tmp_Obs_Data.Pseudorange_2)/(FREQ_BDS_B1I+FREQ_BDS_B3I);;

                Satellites_Obs[PRN]=Tmp_Obs_Data;
                break;
            }
            default:
                break;
        }

    }
}

Rinex304_Observation_Data::Rinex304_Observation_Data(std::ifstream& file, Rinex304_Observation_Head* pHeader_)
{
    pHeader=pHeader_;
    while (1)
    {
        Rinex304_Single_Epoch_Observation_Data Tmp(file, pHeader);
        if(pHeader->First_Obs_Time.GPS_Week==0)
        {
            pHeader->First_Obs_Time=GPST(Tmp.Epoch_Time.year,Tmp.Epoch_Time.month,
                                         Tmp.Epoch_Time.day,Tmp.Epoch_Time.hour,
                                         Tmp.Epoch_Time.min,Tmp.Epoch_Time.sec);
        }

        Rinex_Obs_Data.push_back(Tmp);
        if(file.peek()==EOF)
        {
            if(pHeader->Last_Obs_Time.GPS_Week==0)
            {
                pHeader->Last_Obs_Time=GPST(Tmp.Epoch_Time.year,Tmp.Epoch_Time.month,
                                             Tmp.Epoch_Time.day,Tmp.Epoch_Time.hour,
                                             Tmp.Epoch_Time.min,Tmp.Epoch_Time.sec);
            }
            break;
        }

    }
}



Rinex303_Observation_Head::Rinex303_Observation_Head(std::ifstream& file)
{
    std::string line;
    while (std::getline(file, line)) {
        // RINEX 版本和类型
        if (line.find("RINEX VERSION / TYPE") != std::string::npos)
        {
            version = Read_Double_Data(line,0,9);  // 提取版本号
            // 拷贝类型字段
            std::strncpy(type, line.substr(20, 20).c_str(), 20);  // 拷贝类型字段
        }
        //APPROX POSITION XYZ 位置
        if (line.find("APPROX POSITION XYZ") != std::string::npos)
        {
            Approximate_Position.X = Read_Double_Data(line,0,14);
            Approximate_Position.Y = Read_Double_Data(line,14,14);
            Approximate_Position.Z = Read_Double_Data(line,28,14);
        }

        //TIME OF FIRST OBS
        if (line.find("TIME OF FIRST OBS") != std::string::npos)
        {
            int year=Read_Int_Data(line,0,6);
            int month=Read_Int_Data(line,6,6);
            int day=Read_Int_Data(line,12,6);
            int hour=Read_Int_Data(line,18,6);
            int min=Read_Int_Data(line,24,6);
            double sec= Read_Double_Data(line,30,13);
            First_Obs_Time=GPST(year,month,day,hour,min,sec);
        }

        //TIME OF LAST OBS
        if (line.find("TIME OF LAST OBS") != std::string::npos)
        {
            int year=Read_Int_Data(line,0,6);
            int month=Read_Int_Data(line,6,6);
            int day=Read_Int_Data(line,12,6);
            int hour=Read_Int_Data(line,18,6);
            int min=Read_Int_Data(line,24,6);
            double sec= Read_Double_Data(line,30,13);
            Last_Obs_Time=GPST(year,month,day,hour,min,sec);
        }

        //SYS / # / OBS TYPES 观测值类型
        if (line.find("SYS / # / OBS TYPES") != std::string::npos)
        {
            std::string Sys_Flag=line.substr(0, 1);
            int Num_Of_Type=Read_Int_Data(line,3,3);
            switch (Sys_Flag[0])
            {
                case 'G':
                {
                    //GPS系统
                    int line_num=(Num_Of_Type+12)/13;
                    for (int i=0;i<13;i++)
                    {
                        Sys_Obs_Type['G'].push_back(line.substr(6+4*i+1, 3));
                    }
                    for(int j=1;j<line_num;j++){
                        //续行
                        std::getline(file, line);
                        for (int i = 0; i < Num_Of_Type - 13 * j; i++) {
                            Sys_Obs_Type['G'].push_back(line.substr(6 + 4 * i + 1, 3));
                        }
                    }
                    break;
                }
                case 'C':
                {
                    //北斗系统
                    int line_num=(Num_Of_Type+12)/13;
                    for (int i=0;i<13;i++)
                    {
                        Sys_Obs_Type['C'].push_back(line.substr(6+4*i+1, 3));
                    }
                    for(int j=1;j<line_num;j++){
                        //续行
                        std::getline(file, line);
                        for (int i = 0; i < Num_Of_Type - 13 * j; i++) {
                            Sys_Obs_Type['C'].push_back(line.substr(6 + 4 * i + 1, 3));
                        }
                    }
                    break;
                }
                case 'R':
                {
                    //GLONASS系统
                    int line_num=(Num_Of_Type+12)/13;
                    for (int i=0;i<13;i++)
                    {
                        Sys_Obs_Type['R'].push_back(line.substr(6+4*i+1, 3));
                    }
                    for(int j=1;j<line_num;j++){
                        //续行
                        std::getline(file, line);
                        for (int i = 0; i < Num_Of_Type - 13 * j; i++) {
                            Sys_Obs_Type['R'].push_back(line.substr(6 + 4 * i + 1, 3));
                        }
                    }
                    break;
                }
                case 'E':
                {
                    //GALILEO系统
                    int line_num=(Num_Of_Type+12)/13;
                    for (int i=0;i<13;i++)
                    {
                        Sys_Obs_Type['E'].push_back(line.substr(6+4*i+1, 3));
                    }
                    for(int j=1;j<line_num;j++){
                        //续行
                        std::getline(file, line);
                        for (int i = 0; i < Num_Of_Type - 13 * j; i++) {
                            Sys_Obs_Type['E'].push_back(line.substr(6 + 4 * i + 1, 3));
                        }
                    }
                    break;
                }
                default:
                    break;
            }
        }

        //频点观测历元
        if (line.find("PRN / # OF OBS") != std::string::npos)
        {

        }
        // 文件头结束
        if (line.find("END OF HEADER") != std::string::npos)
        {
            break;
        }
    }
}

Rinex303_Single_Epoch_Observation_Data::Rinex303_Single_Epoch_Observation_Data(std::ifstream& file, Rinex303_Observation_Head* pHeader)
{
    std::string line;
    std::getline(file, line);

    Epoch_Time.year=Read_Int_Data(line,2,4);
    Epoch_Time.month=Read_Int_Data(line,7,2);
    Epoch_Time.day=Read_Int_Data(line,10,2);
    Epoch_Time.hour=Read_Int_Data(line,13,2);
    Epoch_Time.min=Read_Int_Data(line,16,2);
    Epoch_Time.sec= Read_Double_Data(line,18,11);

    Epoch_Flag=Read_Int_Data(line,31,1);

    Num_Of_Satellites=Read_Int_Data(line,32,3);

    for (int i = 0; i < Num_Of_Satellites; i++)
    {
        std::getline(file, line);
        std::string PRN=line.substr(0,3);
        switch (PRN[0])
        {
            case 'G':
            {
                Rinex303_Single_Satellite_Observation_Data Tmp_Obs_Data;
                int offset=3;
                for (auto &Obs_Type:(*pHeader).Sys_Obs_Type['G'])
                {
                    double Obs= Read_Double_Data(line,offset,14);
                    offset+=16;
//                    offset+=14+1;
//                    int Signal_Strength= Read_Int_Data(line,offset,1);
//                    offset+=1;
                    Tmp_Obs_Data.Single_Satellite_Obs[Obs_Type]=Obs;
                }



//                if(Tmp_Obs_Data.Pseudorange_1&&Tmp_Obs_Data.Pseudorange_2)
//                    Tmp_Obs_Data.Pseudorange_Geometry_Free=Tmp_Obs_Data.Pseudorange_1-Tmp_Obs_Data.Pseudorange_2;
//                if(Tmp_Obs_Data.Carrier_Phase_1&&Tmp_Obs_Data.Carrier_Phase_2)
//                    Tmp_Obs_Data.Carrier_Phase_Geometry_Free=C_LIGHT*(Tmp_Obs_Data.Carrier_Phase_1/FREQ_GPS_L1-Tmp_Obs_Data.Carrier_Phase_2/FREQ_GPS_L2);
//                if(Tmp_Obs_Data.Carrier_Phase_1&&Tmp_Obs_Data.Carrier_Phase_2&&Tmp_Obs_Data.Pseudorange_1&&Tmp_Obs_Data.Pseudorange_2)
//                    Tmp_Obs_Data.Melbourne_Wubbena=C_LIGHT/(FREQ_GPS_L1-FREQ_GPS_L2)*(Tmp_Obs_Data.Carrier_Phase_1-Tmp_Obs_Data.Carrier_Phase_2)
//                                                   -(FREQ_GPS_L1*Tmp_Obs_Data.Pseudorange_1+FREQ_GPS_L2*Tmp_Obs_Data.Pseudorange_2)/(FREQ_GPS_L1+FREQ_GPS_L2);;

                Satellites_Obs[PRN]=Tmp_Obs_Data;
                break;
            }
            case 'C':
            {
                Rinex303_Single_Satellite_Observation_Data Tmp_Obs_Data;
                int offset=3;
                for (auto &Obs_Type:(*pHeader).Sys_Obs_Type['C'])
                {
                    double Obs= Read_Double_Data(line,offset,14);
                    offset+=16;
//                    offset+=14+1;
//                    int Signal_Strength= Read_Int_Data(line,offset,1);
//                    offset+=1;
                    Tmp_Obs_Data.Single_Satellite_Obs[Obs_Type]=Obs;
                }

//                Tmp_Obs_Data.Pseudorange_Geometry_Free=Tmp_Obs_Data.Pseudorange_1-Tmp_Obs_Data.Pseudorange_2;
//                Tmp_Obs_Data.Carrier_Phase_Geometry_Free=C_LIGHT*(Tmp_Obs_Data.Carrier_Phase_1/FREQ_BDS_B1I-Tmp_Obs_Data.Carrier_Phase_2/FREQ_BDS_B3I);
//                Tmp_Obs_Data.Melbourne_Wubbena=C_LIGHT/(FREQ_BDS_B1I-FREQ_BDS_B3I)*(Tmp_Obs_Data.Carrier_Phase_1-Tmp_Obs_Data.Carrier_Phase_2)
//                                               -(FREQ_BDS_B1I*Tmp_Obs_Data.Pseudorange_1+FREQ_BDS_B3I*Tmp_Obs_Data.Pseudorange_2)/(FREQ_BDS_B1I+FREQ_BDS_B3I);;

                Satellites_Obs[PRN]=Tmp_Obs_Data;
                break;
            }
            case 'R':
            {
                Rinex303_Single_Satellite_Observation_Data Tmp_Obs_Data;
                int offset=3;
                for (auto &Obs_Type:(*pHeader).Sys_Obs_Type['R'])
                {
                    double Obs= Read_Double_Data(line,offset,14);
                    offset+=16;
//                    offset+=14+1;
//                    int Signal_Strength= Read_Int_Data(line,offset,1);
//                    offset+=1;
                    Tmp_Obs_Data.Single_Satellite_Obs[Obs_Type]=Obs;
                }

//                Tmp_Obs_Data.Pseudorange_Geometry_Free=Tmp_Obs_Data.Pseudorange_1-Tmp_Obs_Data.Pseudorange_2;
//                Tmp_Obs_Data.Carrier_Phase_Geometry_Free=C_LIGHT*(Tmp_Obs_Data.Carrier_Phase_1/FREQ_BDS_B1I-Tmp_Obs_Data.Carrier_Phase_2/FREQ_BDS_B3I);
//                Tmp_Obs_Data.Melbourne_Wubbena=C_LIGHT/(FREQ_BDS_B1I-FREQ_BDS_B3I)*(Tmp_Obs_Data.Carrier_Phase_1-Tmp_Obs_Data.Carrier_Phase_2)
//                                               -(FREQ_BDS_B1I*Tmp_Obs_Data.Pseudorange_1+FREQ_BDS_B3I*Tmp_Obs_Data.Pseudorange_2)/(FREQ_BDS_B1I+FREQ_BDS_B3I);;

                Satellites_Obs[PRN]=Tmp_Obs_Data;
                break;
            }
            case 'E':
            {
                Rinex303_Single_Satellite_Observation_Data Tmp_Obs_Data;
                int offset=3;
                for (auto &Obs_Type:(*pHeader).Sys_Obs_Type['E'])
                {
                    double Obs= Read_Double_Data(line,offset,14);
                    offset+=16;
//                    offset+=14+1;
//                    int Signal_Strength= Read_Int_Data(line,offset,1);
//                    offset+=1;
                    Tmp_Obs_Data.Single_Satellite_Obs[Obs_Type]=Obs;
                }

//                Tmp_Obs_Data.Pseudorange_Geometry_Free=Tmp_Obs_Data.Pseudorange_1-Tmp_Obs_Data.Pseudorange_2;
//                Tmp_Obs_Data.Carrier_Phase_Geometry_Free=C_LIGHT*(Tmp_Obs_Data.Carrier_Phase_1/FREQ_BDS_B1I-Tmp_Obs_Data.Carrier_Phase_2/FREQ_BDS_B3I);
//                Tmp_Obs_Data.Melbourne_Wubbena=C_LIGHT/(FREQ_BDS_B1I-FREQ_BDS_B3I)*(Tmp_Obs_Data.Carrier_Phase_1-Tmp_Obs_Data.Carrier_Phase_2)
//                                               -(FREQ_BDS_B1I*Tmp_Obs_Data.Pseudorange_1+FREQ_BDS_B3I*Tmp_Obs_Data.Pseudorange_2)/(FREQ_BDS_B1I+FREQ_BDS_B3I);;

                Satellites_Obs[PRN]=Tmp_Obs_Data;
                break;
            }
            default:
                break;
        }

    }
}

Rinex303_Observation_Data::Rinex303_Observation_Data(std::ifstream& file, Rinex303_Observation_Head* pHeader_)
{
    pHeader=pHeader_;
    while (1)
    {
        static int index=0;
        Rinex303_Single_Epoch_Observation_Data Tmp(file, pHeader);
        if(pHeader->First_Obs_Time.GPS_Week==0)
        {
            pHeader->First_Obs_Time=GPST(Tmp.Epoch_Time.year,Tmp.Epoch_Time.month,
                                         Tmp.Epoch_Time.day,Tmp.Epoch_Time.hour,
                                         Tmp.Epoch_Time.min,Tmp.Epoch_Time.sec);
        }

        //判断单颗卫星连续观测时间段 已弃用
        /*
        for (auto &pair:Tmp.Satellites_Obs)
        {

            if(Time_Range.count(pair.first)==0)
            {
                Time_Range[pair.first].push_back({index,index});
            }
            else{
                auto &[l, r] = Time_Range[pair.first].back();
                if(index==r+1&&pair.second.Melbourne_Wubbena!=0&&pair.second.Carrier_Phase_Geometry_Free!=0&&pair.second.Pseudorange_Geometry_Free!=0)
                {
                    r=index;
                } else{
                    Time_Range[pair.first].push_back({index,index});
                }
            }
        }
         */
        Rinex_Obs_Data.push_back(Tmp);
        if(file.peek()==EOF)
        {
            if(pHeader->Last_Obs_Time.GPS_Week==0)
            {
                pHeader->Last_Obs_Time=GPST(Tmp.Epoch_Time.year,Tmp.Epoch_Time.month,
                                            Tmp.Epoch_Time.day,Tmp.Epoch_Time.hour,
                                            Tmp.Epoch_Time.min,Tmp.Epoch_Time.sec);
            }
            break;
        }
        index++;
    }
}
