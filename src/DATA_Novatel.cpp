//
// Created by asus on 2025/4/29.
//

#include "DATA_Novatel.h"

//std::sort比较函数
bool GPS_Ephem_Compare(const Novatel_Message_GPSEPHEM& a, const Novatel_Message_GPSEPHEM& b)
{
    if(a.Week!=b.Week)
        return a.Week<b.Week;
    if(a.Toe!=b.Toe)
        return a.Toe<b.Toe;
    return a.PRN<b.PRN;
}
bool BDS_Ephem_Compare(const Novatel_Message_BDSEPHEMERIS& a, const Novatel_Message_BDSEPHEMERIS& b)
{
    if(a.Week!=b.Week)
        return a.Week<b.Week;
    if(a.Toe!=b.Toe)
        return a.Toe<b.Toe;
    return a.Satellite_ID<b.Satellite_ID;
}








Novatel_Data::Novatel_Data(std::ifstream &file)
{
    unsigned long long cnt=0;

    //确保文件开头为同步符
    Novatel_Binary_Header_SyncSearch(file);
    cnt=file.tellg();

    while (true)
    {
        std::streampos pos_ = file.tellg();
        //读完文件 结束循环
        if (file.peek() == EOF) break;

        Novatel_Data_Binary_Header Tmp_Data_Binary_Header(file);
        //检查长度是否匹配，若不匹配则直接跳到下一个同步头
        if(Novatel_Binary_MessageLengthCheck(file,Tmp_Data_Binary_Header.Message_Length)==false)
        {
            cnt=file.tellg();
            continue;
        }
        file.seekg(pos_);

        cnt=cnt+Tmp_Data_Binary_Header.Hearder_Length+4+Tmp_Data_Binary_Header.Message_Length;
        switch (Tmp_Data_Binary_Header.Message_ID)
        {
            //C++ 不允许在 case 标签后直接进入一个新的变量声明或初始化语句，
            //除非这些语句被包含在 {} 的作用域块中。
            //如果在其中直接声明变量，会违反 C++ 的作用域规则。
            case 7://Message ID 7 GPSEPHME
            {
                vData_Header.push_back(Tmp_Data_Binary_Header);

                std::streampos pos = file.tellg();

                if(CalcMessageCRC32(file, Tmp_Data_Binary_Header.Message_Length))
                {
                    vData_Header.pop_back();
                    printf("CRC ERROR\n");
                    break;
                }
                file.seekg(pos);
                file.seekg(28, std::ios::cur);
                Novatel_Message_GPSEPHEM Tmp_Message_GPSEPHEM(file, &vData_Header.back());
                vMessage_GPSEPHEM.push_back(Tmp_Message_GPSEPHEM);

                //GPS_PVT(Tmp_Message_GPSEPHEM, *this);

                break;
            }
            case 42://Message ID 42 BESTPOS
            {
                vData_Header.push_back(Tmp_Data_Binary_Header);

                std::streampos pos = file.tellg();

                if(CalcMessageCRC32(file, Tmp_Data_Binary_Header.Message_Length))
                {
                    vData_Header.pop_back();
                    printf("CRC ERROR\n");
                    break;
                }
                file.seekg(pos);
                file.seekg(28, std::ios::cur);
                Novatel_Message_BESTPOS Tmp_Message_BESTPOS(file, &vData_Header.back());

                vMessage_BESTPOS.push_back(Tmp_Message_BESTPOS);
                break;
            }
            case 43://Message_ID 43 RANGE
            {
                vData_Header.push_back(Tmp_Data_Binary_Header);

                std::streampos pos = file.tellg();

                if(CalcMessageCRC32(file, Tmp_Data_Binary_Header.Message_Length))
                {
                    vData_Header.pop_back();
                    printf("CRC ERROR\n");
                    break;
                }
                file.seekg(pos);
                file.seekg(28, std::ios::cur);
                Novatel_Message_RANGE Tmp_Message_RANGE(file, &vData_Header.back(),*this);
                vMessage_RANGE.push_back(Tmp_Message_RANGE);
                break;
            }
            case 47://Message_ID 47 PSRPOS
            {
                vData_Header.push_back(Tmp_Data_Binary_Header);

                std::streampos pos = file.tellg();

                if(CalcMessageCRC32(file, Tmp_Data_Binary_Header.Message_Length))
                {
                    vData_Header.pop_back();
                    printf("CRC ERROR\n");
                    break;
                }
                file.seekg(pos);
                file.seekg(28, std::ios::cur);
                Novatel_Message_PSRPOS Tmp_Message_PSRPOS(file, &vData_Header.back());
                vMessage_PSRPOS.push_back(Tmp_Message_PSRPOS);
                break;
            }
            case 172://Message ID 172 AVEPOS
            {
                vData_Header.push_back(Tmp_Data_Binary_Header);

                std::streampos pos = file.tellg();

                if(CalcMessageCRC32(file, Tmp_Data_Binary_Header.Message_Length))
                {
                    vData_Header.pop_back();
                    printf("CRC ERROR\n");
                    break;
                }
                file.seekg(pos);
                file.seekg(28, std::ios::cur);
                Novatel_Message_AVEPOS Tmp_Message_AVEPOS(file, &vData_Header.back());
                vMessage_AVEPOS.push_back(Tmp_Message_AVEPOS);
                break;
            }
            case 469://Message ID 469 PDPPOS
            {
                vData_Header.push_back(Tmp_Data_Binary_Header);

                std::streampos pos = file.tellg();

                if(CalcMessageCRC32(file, Tmp_Data_Binary_Header.Message_Length))
                {
                    vData_Header.pop_back();
                    printf("CRC ERROR\n");
                    break;
                }
                file.seekg(pos);
                file.seekg(28, std::ios::cur);
                Novatel_Message_PDPPOS Tmp_Message_PDPPOS(file, &vData_Header.back());
                vMessage_PDPPOS.push_back(Tmp_Message_PDPPOS);
                break;
            }
            case 1696://Message ID 1696 BDSEPHEMERIS
            {
                vData_Header.push_back(Tmp_Data_Binary_Header);

                std::streampos pos = file.tellg();

                if(CalcMessageCRC32(file, Tmp_Data_Binary_Header.Message_Length))
                {
                    vData_Header.pop_back();
                    printf("CRC ERROR\n");
                    break;
                }
                file.seekg(pos);
                file.seekg(28, std::ios::cur);
                Novatel_Message_BDSEPHEMERIS Tmp_Message_BDSEPHEMERIS(file, &vData_Header.back());
                vMessage_BDSEPHEMERIS.push_back(Tmp_Message_BDSEPHEMERIS);
                //BDS_PVT(Tmp_Message_BDSEPHEMERIS,*this);
                break;
            }
            default:
            {
                printf("Message ID is %d\n",Tmp_Data_Binary_Header.Message_ID);
                break;
            }
        }
//        std::cout << "Before header: pos=" << file.tellg() << std::endl;
//        printf("Cnt is %llu\n",cnt);
//        printf("%s\n",std::string(66, '-').c_str());
    }
    std::sort(vMessage_GPSEPHEM.begin(),vMessage_GPSEPHEM.end(),GPS_Ephem_Compare);
    std::sort(vMessage_BDSEPHEMERIS.begin(),vMessage_BDSEPHEMERIS.end(),BDS_Ephem_Compare);

    double Trop=Hopfield(18.389*DEG2RAD,41.933);
    printf("%lf\n",Trop);

    for(auto &obs:Observation)
        {
            Single_Point_Pusitioning(*this,obs);
        }
//    Single_Point_Pusitioning(*this,Observation[8]);



//    Cartesian_XYZ_Coord Test(-2267805.5272 ,5009342.8364,3220991.8580);
//    BLH_Coord BLH_Test= XYZ2BLH(Test);
//    printf("B:%.12lf\tL:%.12lf\tH:%.12lf\n",BLH_Test.B*180/PI,BLH_Test.L*180/PI,BLH_Test.H);
//    Cartesian_XYZ_Coord Test_2(2487791.960 , 21454452.060 , 15026178.221);
//    Cartesian_XYZ_Coord Refer(-2267805.5272 ,5009342.8364,3220991.8580);
//    ENU ENU_Test= XYZ2ENU(Test_2,Refer);
//    double E=ENU_Test.E;
//    double N=ENU_Test.N;
//    double U=ENU_Test.U;
//    double Elevation_Angle=std::atan2(U,std::sqrt((E*E+N*N)));
//    printf("E:%.12lf\tN:%.12lf\tU:%.12lf\nElevation_Angle:%.3lf\n"
//           ,ENU_Test.E,ENU_Test.N,ENU_Test.U,Elevation_Angle*180/PI);
//
//
//    double Trop=Hopfield(18.389*DEG2RAD,41.933);
//    printf("%lf",Trop);

}
