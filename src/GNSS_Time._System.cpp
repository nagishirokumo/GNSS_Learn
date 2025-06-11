//
// Created by asus on 2025/4/17.
//
#include <cmath>
#include "../include/GNSS_Time_System.h"
#include <iostream>

void MJD::output() const
{
    std::wcout<<this->mjd<<std::endl;
}

UTC MJD::MJD2UTC()
{
    unsigned long jd = this->mjd + 2400000.5;

    int Z = static_cast<int>(jd + 0.5);
    double F = jd + 0.5 - Z;

    int A = Z;
    if (Z >= 2299161) {
        int alpha = (Z - 1867216.25) / 36524.25;
        A += 1 + alpha - alpha / 4;
    }

    int B = A + 1524;
    int C = (B - 122.1) / 365.25;
    int D = 365.25 * C;
    int E = (B - D) / 30.6001;

    double day = B - D - static_cast<int>(30.6001 * E) + F;
    int day_int = static_cast<int>(day);

    int month = (E < 14) ? (E - 1) : (E - 13);
    int year = (month > 2) ? (C - 4716) : (C - 4715);

    double day_frac = day - day_int;
    double total_seconds = day_frac * 86400.0;

    int hour = total_seconds / 3600;
    int min = (total_seconds - hour * 3600) / 60;
    double sec = total_seconds - hour * 3600 - min * 60;

    UTC temp(year,month,day_int,hour,min,sec);
    temp.output();
    return temp;
}

void UTC::output()
{
    printf("%d/%d/%d/%d/%d/%d.%lf",year,month,day,hour,min,sec);
}

GPST BDT2GPST(BDT BDTime)
{
    GPST GPSTime;
    GPSTime.GPS_Week=BDTime.BDS_Week+1356;
    GPSTime.GPS_Second=BDTime.BDS_Second+14;
    if (GPSTime.GPS_Second>=604800)
    {
        GPSTime.GPS_Second-=604800;
        GPSTime.GPS_Week+=1;
    }
    return GPSTime;
}

BDT GPST2BDT(GPST GPSTime)
{
    BDT BDTime;
    BDTime.BDS_Week=GPSTime.GPS_Week-1356;
    BDTime.BDS_Second=GPSTime.GPS_Second-14;
    if (GPSTime.GPS_Second<=0)
    {
        GPSTime.GPS_Second+=604800;
        GPSTime.GPS_Week-=1;
    }
    return BDTime;
}


