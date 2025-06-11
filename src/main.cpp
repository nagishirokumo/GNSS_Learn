#include <iostream>
#include "GNSS_Time_System.h"
#include "GNSS_Coord_System.h"
#include "Data_RINEX303.h"
#include "DATA_Novatel.h"
#include "sockets.h"
#include <windows.h>
#include <io.h>
#include <fcntl.h>


int Novatel_main()
{
    std::ifstream file("../resources/NovatelOEM20211114-01.log",std::ios::in|std::ios::binary);
    Novatel_Data Novatel_Data(file);
    return 0;
}
int RINEX_Navi_main()
{
    std::ifstream file("../resources/brdm3350.19p");
    Rinex_Navigation_Message_Data RINEX_Data(file, 0);
    std::wcout<<RINEX_Data.vData_Body.size()<<std::endl;
    return 0;
}
int RINEX_Obs_main()
{
    std::ifstream file("../resources/JFNG00CHN_R_20200010000_01D_30S_MO.20o");
    Rinex_Observation_Head Obs_Head(file);
    Rinex_Observation_Data Obs_Data(file,&Obs_Head);
    Rinex_Observation_Epoch_Differential Obs_Epoch_Differential(Obs_Data);
    Write2TXT(Obs_Data);
    Write2TXT(Obs_Epoch_Differential);
    return 0;
}

bool Socket_Test()
{
    SOCKET my_soc;
    char IP[]="8.148.22.229";
    uint16_t port=4002;
    if(OpenSocket(my_soc,IP,port)== false)
    {
        printf( "The ip %s was not opened\n", IP );
        return false;
    }
    std::string output_path="Socket_data.log";
    saveBinaryData(my_soc,output_path);
    return true;
}

int main()
{
    SetConsoleOutputCP(CP_UTF8);
//    Novatel OEM7
    Novatel_main();
//    Socket_Test();


//    RINEX303
//    RINEX_Navi_main();
//    RINEX_Obs_main();
    return 0;
}

