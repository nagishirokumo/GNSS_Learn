//
// Created by asus on 2025/6/5.
//

#include "DATA_Novatel.h"

void Uint8_read(std::ifstream& file, uint8_t* puint8)
{
    file.read(reinterpret_cast<char*>(puint8), sizeof(*puint8));
    return;
}
void Int8_read(std::ifstream& file, int8_t *pint8)
{
    file.read(reinterpret_cast<char*>(pint8), sizeof(*pint8));
    return;
}
void Uint16_read(std::ifstream& file, uint16_t* puint16)
{
    file.read(reinterpret_cast<char*>(puint16), sizeof(*puint16));
    return;
}
void Int16_read(std::ifstream& file, uint16_t* pint16)
{
    file.read(reinterpret_cast<char*>(pint16), sizeof(*pint16));
    return;
}
void Uint32_read(std::ifstream& file, uint32_t* puint32)
{
    file.read(reinterpret_cast<char*>(puint32), sizeof(*puint32));
    return;
}
void Int32_read(std::ifstream& file, int32_t* pint32)
{
    file.read(reinterpret_cast<char*>(pint32), sizeof(*pint32));
    return;
}
void Float32_read(std::ifstream& file, float_t* pfloat32)
{
    file.read(reinterpret_cast<char*>(pfloat32), sizeof(*pfloat32));
    return;
}
void Int64_read(std::ifstream& file, int64_t* pint64)
{
    file.read(reinterpret_cast<char*>(pint64), sizeof(*pint64));
    return;
}
void Double64_read(std::ifstream& file, double_t* pdouble64)
{
    file.read(reinterpret_cast<char*>(pdouble64), sizeof(*pdouble64));
    return;
}

//CRC循环冗余校验
unsigned long CRC32Value(int i)
{
    unsigned long ulCRC = i;
    for (int j = 8; j > 0; j--) {
        if (ulCRC & 1)
            ulCRC = (ulCRC >> 1) ^ 0xEDB88320u;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}
bool CalcMessageCRC32(std::ifstream &file, uint16_t Message_Length)
{
    int len=Message_Length+28;
    unsigned char buffer[len];
    file.read(reinterpret_cast<char*>(buffer), len);

    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    unsigned long expectedCRC;
    file.read(reinterpret_cast<char*>(&expectedCRC), 4);
    for (int i = 0; i < len; i++)
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int) ulCRC ^ buffer[i] ) & 0xFF );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
//    int i,j;
//    unsigned int crc=0;
//    for (i=0;i<len;i++)
//    {
//        crc^=buffer[i];
//        for (j=0;j<8;j++)
//        {
//            if (crc & 1)
//                crc = (crc >> 1) ^ 0xEDB88320u;
//            else
//                crc >>= 1;
//        }
//    }
    return ulCRC != expectedCRC;
}

//处理文件开始同步头找不到的问题
bool Novatel_Binary_Header_SyncSearch(std::ifstream& file)
{
    std::streampos pos_ = file.tellg();
    while(1)
    {
        std::streampos pos = file.tellg();

        char buf[3];
        file.read(buf, 3);
        if ((unsigned char)buf[0] == 0xAA && (unsigned char)buf[1] == 0x44 && (unsigned char)buf[2] == 0x12)
        {
            file.seekg(pos);

            if((pos-pos_)!=0)
            {
//                std::cerr << "Setoff " << pos - pos_ << std::endl;
//                std::cerr.flush();
                printf("Setoff %lld\n",pos - pos_);
            }
            return 1;
        }
        file.seekg(pos);
        file.seekg(1, std::ios::cur);

    }
}
//检查消息长度是否符合实际两个同步头间距
bool Novatel_Binary_MessageLengthCheck(std::ifstream& file,unsigned short length)
{
    std::streampos pos = file.tellg();
    //偏移到理论下一组同步头所在位置,因为本函数在读取完头部后使用，偏移量不含头部长度28
    file.seekg(length+4, std::ios::cur);
    if(file.peek() == EOF)
    {
        return 1;
    }
    //检查同步头
    char buf[3];
    file.read(buf, 3);
    if ((unsigned char)buf[0] == 0xAA && (unsigned char)buf[1] == 0x44 && (unsigned char)buf[2] == 0x12)
    {
        file.seekg(pos);
        return 1;
    }
    else
    {
        file.seekg(pos);
//        std::cerr << "The Message Length Error" << std::endl;
//        std::cerr.flush();
        printf("The Message Length Error\n");
        Novatel_Binary_Header_SyncSearch(file);
        return 0;
    }
}

//检查同步头
bool Novatel_Binary_Header_SyncCheck(std::ifstream& file,uint8_t* pSync)
{
    for(int i=0;i<3;i++) { Uint8_read(file, pSync + i); }
    if(pSync[0] == 0xAA && pSync[1] == 0x44 && pSync[2] == 0x12)
    {
//        std::cout<<"the file is Novatel_Data_Binary"<<std::endl;
//        printf("Sync is %02X%02X%02X\n",pSync[0],pSync[1],pSync[2]);
        return 1;
    }
    else{
        std::cout<<"the file is not Novatel_Data_Binary"<<std::endl;
        printf("Sync is %02X%02X%02X\n",pSync[0],pSync[1],pSync[2]);
        return 0;
    }
}

//构造函数
Novatel_Data_Binary_Header::Novatel_Data_Binary_Header(std::ifstream& file)
{
    if(Novatel_Binary_Header_SyncCheck(file,Sync)==false)
    {
        std::cerr<<"The file type error\n"<<std::endl;
    }
    Uint8_read(file, &Hearder_Length);
    Uint16_read(file, &Message_ID);
    Int8_read(file, &Message_Type);
    Uint8_read(file, &Port_Address);
    Uint16_read(file,&Message_Length);
    Uint16_read(file,&Sequence);
    Int8_read(file,&Idle_Time);
    Int8_read(file,&Time_Status);
    Uint16_read(file,&GPS_Week);
    Uint32_read(file,&Milliseconds);
    Uint32_read(file,&Receiver_Status);
    Uint16_read(file,&Reserved);
    Uint16_read(file,&Receiver_Version);
    if(Hearder_Length!=0x1C)
    {
        std::cerr<<"The Hearder_Length is "<<Hearder_Length<<std::endl;
    }
//    printf("Message_ID %d\n",Message_ID);
//    printf("Message_Length %d\n",Message_Length);
}

Novatel_Message_RANGE::Novatel_Message_RANGE(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_)
        :pHeader(pHeader_)
{
    Uint32_read(file,&Num_Of_Observations);
//    printf("Num_Of_Observations %d\n",Num_Of_Observations);

    unsigned int cnt=Num_Of_Observations;
    while(cnt--)
    {
        Novatel_Message_RANGE_DataBlock Tmp_DataBlock(file);
        vRange_DataBlock.push_back(Tmp_DataBlock);
    }

    Uint32_read(file,&CRC);
}
Novatel_Message_RANGE_DataBlock::Novatel_Message_RANGE_DataBlock(std::ifstream& file)
{
    Uint16_read(file,&PRN);
    Uint16_read(file,&GLONASS_Frequency);
    Double64_read(file,&Psr);
    Float32_read(file,&Psr_Sigma);
    Double64_read(file,&Adr);
    Float32_read(file,&Adr_Sigma);
    Float32_read(file,&Dopp);
    Float32_read(file,&C_No);
    Float32_read(file,&Locktime);
    Uint32_read(file,&Ch_tr_status);
}
Novatel_Message_RANGE::Novatel_Message_RANGE(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_, Novatel_Data &Novatel_Data)
        :pHeader(pHeader_)
{
    Uint32_read(file,&Num_Of_Observations);
//    printf("Num_Of_Observations %d\n",Num_Of_Observations);

    unsigned int cnt=Num_Of_Observations;

    Novatel_Single_Epoch_OBSERVATION Epoch_Obs;
    Epoch_Obs.GPS_Time.GPS_Week=(*pHeader_).GPS_Week;
    Epoch_Obs.GPS_Time.GPS_Second=(*pHeader_).Milliseconds*1.0/1000.0;

    while(cnt--)
    {
        Novatel_Message_RANGE_DataBlock Tmp_DataBlock(file);

        int System_Flag=Tmp_DataBlock.Ch_tr_status;

        System_Flag=(System_Flag>>16)&0xF; //取出第五位 0:GPS 4:BeiDou

        switch (System_Flag) {
            case 0:
            {
                Novatel_OBSERVATION Obs(pHeader_,Tmp_DataBlock);
                Epoch_Obs.Single_Epoch_GPS_Observation[Tmp_DataBlock.PRN].push_back(Obs);
                break;
            }
            case 4:
            {
                Novatel_OBSERVATION Obs(pHeader_,Tmp_DataBlock);
                Epoch_Obs.Single_Epoch_BDS_Observation[Tmp_DataBlock.PRN].push_back(Obs);
                break;
            }
        }

        vRange_DataBlock.push_back(Tmp_DataBlock);
    }
    Novatel_Data.Observation.push_back(Epoch_Obs);
    Uint32_read(file,&CRC);
}

Novatel_OBSERVATION::Novatel_OBSERVATION(Novatel_Data_Binary_Header* pHeader,Novatel_Message_RANGE_DataBlock& DataBlock)
{
//    GPS_Time.GPS_Week=(*pHeader).GPS_Week;
//    GPS_Time.GPS_Second=1.0*(*pHeader).Milliseconds/1000;
//    GLONASS_Frequency=DataBlock.GLONASS_Frequency;
    Psr=DataBlock.Psr;
    Psr_Sigma=DataBlock.Psr_Sigma;
    Adr=DataBlock.Adr;
    Adr_Sigma=DataBlock.Adr_Sigma;
    Dopp=DataBlock.Dopp;
    C_No=DataBlock.C_No;
//    Locktime=DataBlock.Locktime;
//    Ch_tr_status=DataBlock.Ch_tr_status;
    System_Flag=(DataBlock.Ch_tr_status>>16)&0xF;
    Signal_Type=(DataBlock.Ch_tr_status>>21)&0x1F; //提取第 21 到第 25 位
}


Novatel_Message_PSRPOS::Novatel_Message_PSRPOS(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_)
        :pHeader(pHeader_)
{
    Int32_read(file,&Sol_Status);
    Int32_read(file,&Pos_Type);
    Double64_read(file,&Latitude);
    Double64_read(file,&Longitude);
    Double64_read(file,&Height);
    Float32_read(file,&Undulation);
    Int32_read(file,&Datum_ID);
    Float32_read(file,&Latitude_Sigma);
    Float32_read(file,&Longitude_Sigma);
    Float32_read(file,&Height_Sigma);
    Int8_read(file,Base_Station_ID);
    Int8_read(file,Base_Station_ID+1);
    Int8_read(file,Base_Station_ID+2);
    Int8_read(file,Base_Station_ID+3);
    Float32_read(file,&Differential_Age);
    Float32_read(file,&Solution_Age);
    Uint8_read(file,&SVs);
    Uint8_read(file,&SolnSVs);
    Uint8_read(file,Reserved);
    Uint8_read(file,Reserved+1);
    Uint8_read(file,Reserved+2);
    Uint8_read(file,&Ext_Sol_Stat);
    Uint8_read(file,&Gal_BD_Sig_mask);
    Uint8_read(file,&GPS_GLO_Sig_mask);
    Uint32_read(file,&CRC);
//    printf("Base_Station_ID %c%c%c%c\n",Base_Station_ID[0],Base_Station_ID[1],
//           Base_Station_ID[2],Base_Station_ID[3]);

//    printf("Latitude %.9lf\n",Latitude);
//    printf("Longitude %.9lf\n",Longitude);
//    printf("Height %.9lf\n",Height);
//    printf("CRC %08X\n",CRC);
}
Novatel_Message_BESTPOS::Novatel_Message_BESTPOS(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_)
        :pHeader(pHeader_)
{
    Int32_read(file,&Sol_status);
    Int32_read(file,&Pos_Type);
    Double64_read(file,&Latitude);
    Double64_read(file,&Longitude);
    Double64_read(file,&Height);
    Float32_read(file,&Undulation);
    Int32_read(file,&Datum_ID);
    Float32_read(file,&Latitude_Sigma);
    Float32_read(file,&Longitude_Sigma);
    Float32_read(file,&Height_Sigma);
    Int8_read(file,Base_Station_ID);
    Int8_read(file,Base_Station_ID+1);
    Int8_read(file,Base_Station_ID+2);
    Int8_read(file,Base_Station_ID+3);
    Float32_read(file,&Differential_Age);
    Float32_read(file,&Solution_Age);
    Uint8_read(file,&SVs);
    Uint8_read(file,&SolnSVs);
    Uint8_read(file,&SolnL1SVs);
    Uint8_read(file,&SolnMultiSVs);
    Uint8_read(file,&Reserved);
    Uint8_read(file,&Ext_Sol_Stat);
    Uint8_read(file,&Gal_BD_Sig_mask);
    Uint8_read(file,&GPS_GLO_Sig_mask);
    Uint32_read(file,&CRC);

//    printf("Latitude %.9lf\n",Latitude);
//    printf("Longitude %.9lf\n",Longitude);
//    printf("Height %.9lf\n",Height);
//    printf("CRC %08X\n",CRC);
}
Novatel_Message_GPSEPHEM::Novatel_Message_GPSEPHEM(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_)
        :pHeader(pHeader_)
{
    Uint32_read(file,&PRN);
    Double64_read(file,&Tow);
    Uint32_read(file,&Health);
    Uint32_read(file,&IDOE1);
    Uint32_read(file,&IDOE2);
    Uint32_read(file,&Week);
    Uint32_read(file,&Z_Week);
    Double64_read(file,&Toe);
    Double64_read(file,&A);
    Double64_read(file,&Delta_N);
    Double64_read(file,&M0);
    Double64_read(file,&e);
    Double64_read(file,&omega);
    Double64_read(file,&Cuc);
    Double64_read(file,&Cus);
    Double64_read(file,&Crc);
    Double64_read(file,&Crs);
    Double64_read(file,&Cic);
    Double64_read(file,&Cis);
    Double64_read(file,&I0);
    Double64_read(file,&I_Dot);
    Double64_read(file,&OMEGA0);
    Double64_read(file,&OMEGADOT);
    Uint32_read(file,&Idoc);
    Double64_read(file,&Toc);
    Double64_read(file,&Tgd);
    Double64_read(file,&sa0);
    Double64_read(file,&sa1);
    Double64_read(file,&sa2);
    Uint32_read(file,&AS);
    Double64_read(file,&N);
    Double64_read(file,&URA);
    Uint32_read(file,&CRC);

    sqrtA= sqrt(A);
//    printf("Tow %lf\n",Tow);
//    printf("Toe %lf\n",Toe);
//    printf("A %lf\n",A);
}
Novatel_Message_BDSEPHEMERIS::Novatel_Message_BDSEPHEMERIS(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_)
        :pHeader(pHeader_)
{
    Uint32_read(file,&Satellite_ID);
    Uint32_read(file,&Week);
    Double64_read(file,&URA);
    Uint32_read(file,&Health1);
    Double64_read(file,&Tgd1);
    Double64_read(file,&Tgd2);
    Uint32_read(file,&AODC);
    Uint32_read(file,&Toc);
    Double64_read(file,&sa0);
    Double64_read(file,&sa1);
    Double64_read(file,&sa2);
    Uint32_read(file,&AODE);
    Uint32_read(file,&Toe);
    Double64_read(file,&Root_A);
    Double64_read(file,&e);
    Double64_read(file,&omega);
    Double64_read(file,&Delta_N);
    Double64_read(file,&M0);
    Double64_read(file,&OMEGA0);
    Double64_read(file,&OMEGADOT);
    Double64_read(file,&I0);
    Double64_read(file,&I_Dot);
    Double64_read(file,&Cuc);
    Double64_read(file,&Cus);
    Double64_read(file,&Crc);
    Double64_read(file,&Crs);
    Double64_read(file,&Cic);
    Double64_read(file,&Cis);
    Uint32_read(file,&CRC);

    A=Root_A*Root_A;

//    printf("Tow %d\n",Satellite_ID);
//    printf("Toe %d\n",Toe);
//    printf("A %lf\n",Root_A*Root_A);
}
Novatel_Message_AVEPOS::Novatel_Message_AVEPOS(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_)
        :pHeader(pHeader_)
{
    Double64_read(file,&Latitude);
    Double64_read(file,&Longitude);
    Double64_read(file,&Height);
    Float32_read(file,&Latitude_Sigma);
    Float32_read(file,&Longitude_Sigma);
    Float32_read(file,&Height_Sigma);
    Int32_read(file,&Pos_Ave);
    Uint32_read(file,&Ave_Time);
    Uint32_read(file,&Samples);
    Uint32_read(file,&CRC);
}
Novatel_Message_PDPPOS::Novatel_Message_PDPPOS(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_)
        :pHeader(pHeader_)
{
    Int32_read(file,&Sol_status);
    Int32_read(file,&Pos_Type);
    Double64_read(file,&Latitude);
    Double64_read(file,&Longitude);
    Double64_read(file,&Height);
    Float32_read(file,&Undulation);
    Int32_read(file,&Datum_ID);
    Float32_read(file,&Latitude_Sigma);
    Float32_read(file,&Longitude_Sigma);
    Float32_read(file,&Height_Sigma);
    Int8_read(file,Base_Station_ID);
    Int8_read(file,Base_Station_ID+1);
    Int8_read(file,Base_Station_ID+2);
    Int8_read(file,Base_Station_ID+3);
    Float32_read(file,&Differential_Age);
    Float32_read(file,&Solution_Age);
    Uint8_read(file,&Sats);
    Uint8_read(file,&SolnSats);
    Uint8_read(file,Reserved);
    Uint8_read(file,Reserved+1);
    Uint8_read(file,Reserved+2);
    Uint8_read(file,&Ext_Sol_Stat);
    Uint8_read(file,&Gal_BD_Sig_mask);
    Uint8_read(file,&GPS_GLO_Sig_mask);
    Uint32_read(file,&CRC);
//    printf("Base_Station_ID %c%c%c%c\n",Base_Station_ID[0],Base_Station_ID[1],
//           Base_Station_ID[2],Base_Station_ID[3]);

//    printf("Latitude %.9lf\n",Latitude);
//    printf("Longitude %.9lf\n",Longitude);
//    printf("Height %.9lf\n",Height);
//    printf("CRC %08X\n",CRC);
}