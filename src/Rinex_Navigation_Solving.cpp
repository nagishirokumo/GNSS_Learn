//
// Created by asus on 2025/7/3.
//
#include "Data_Rinex.h"

Rinex_Navigation_Message_Data::Rinex_Navigation_Message_Data(std::ifstream& file, int time_cur, double GM)
{
    this->GM =GM;
    Rinex_Navigation_Message_Header_Read(file, this->Data_Head);
    Rinex_NavBlock_Read(file, this->vData_Body);
    Positioning(*this);
    Write2CSV(*this);
}

Rinex_Navigation_Message_Result::Rinex_Navigation_Message_Result(std::string sPRN_): sPRN(sPRN_){};

void Rinex_Navigation_Message_Header_Read(std::ifstream& file, Navi_Data_Head& header) {
    std::string line;
    while (std::getline(file, line)) {
        // RINEX 版本和类型
        if (line.find("RINEX VERSION / TYPE") != std::string::npos)
        {
            header.version = std::stod(line.substr(0, 9));  // 提取版本号
            std::cout<<header.version;
            // 拷贝类型字段（确保在长度内）
            if (line.length() >= 40) {
                std::strncpy(header.type, line.substr(20, 20).c_str(), 20);  // 拷贝类型字段
            }
        }
            // 北斗时间修正（BDUT）
        else if (line.find("BDUT") != std::string::npos)
        {
            std::istringstream iss(line.substr(4, 56));
            iss >> header.BDT_CORR[0] >> header.BDT_CORR[1] >> header.BDT_CORR[2] >> header.BDT_CORR[3];
        }
            // GPS时间修正（GPGA）
        else if (line.find("GPGA") != std::string::npos)
        {
            std::istringstream iss(line.substr(4, 56));
            iss >> header.GPST_CORR[0] >> header.GPST_CORR[1] >> header.GPST_CORR[2] >> header.GPST_CORR[3];
        }
            // 跳秒
        else if (line.find("LEAP SECONDS") != std::string::npos)
        {
            header.leap_seconds = std::stoi(line.substr(0, 6));
        }
            // 文件头结束
        else if (line.find("END OF HEADER") != std::string::npos)
        {
            break;
        }
    }
    std::cout << "\nRINEX Version: " << header.version << "\nTYPE: " << header.type << "\n";
}
//判断是否全空
bool isAllSpaces(const std::string& str) {
    return std::all_of(str.begin(), str.end(), [](char c) {
        return c == ' ';
    });
}
// 提取double数据
double Read_Field(const std::string& line, int idx)
{
    std::string field;
    if (line.size() >= 4+idx * 19) {
        field = line.substr(4+idx * 19, 19);
    } else {
        return 0.0;
    }

    if(isAllSpaces(field))
        return 0.0;
    else
        return std::stod(field);
}
// 解析每组导航数据（共8行），填入 Navi_Data_Body 结构体
void Rinex_NavBlock_Read(std::ifstream& file, std::vector<Navi_Data_Body>& navs) {
    std::string line;
    while (std::getline(file, line)) {
        if (line.length() < 4) continue; // 如果行太短就跳过

        Navi_Data_Body nav;

        // 提取卫星号（例如 "G01"）
        nav.sPRN = line.substr(0, 3);
        if(nav.sPRN[0]=='G'||nav.sPRN[0]=='C')
        {// 提取时间和卫星钟差、偏、漂
            std::istringstream iss(line.substr(4, 19));
            iss >> nav.TOC_Y >> nav.TOC_M >> nav.TOC_D >> nav.TOC_H >> nav.TOC_Min >> nav.TOC_Sec;
            nav.sa0 = Read_Field(line, 1);
            nav.sa1 = Read_Field(line, 2);
            nav.sa2 = Read_Field(line, 3);

            // 接下来读取剩下7行，每行4个字段
            std::getline(file, line);
            nav.IODE = Read_Field(line, 0);
            nav.Crs = Read_Field(line, 1);
            nav.deltan = Read_Field(line, 2);
            nav.M0 = Read_Field(line, 3);
            std::getline(file, line);
            nav.Cuc = Read_Field(line, 0);
            nav.e = Read_Field(line, 1);
            nav.Cus = Read_Field(line, 2);
            nav.sqrtA = Read_Field(line, 3);
            std::getline(file, line);
            nav.TOE = Read_Field(line, 0);
            nav.Cic = Read_Field(line, 1);
            nav.OMEGA0 = Read_Field(line, 2);
            nav.Cis = Read_Field(line, 3);
            std::getline(file, line);
            nav.i0 = Read_Field(line, 0);
            nav.Crc = Read_Field(line, 1);
            nav.omega = Read_Field(line, 2);
            nav.OMEGADOT = Read_Field(line, 3);
            std::getline(file, line);
            nav.IDOT = Read_Field(line, 0);
            nav.L2code = Read_Field(line, 1);
            nav.GPSweek = Read_Field(line, 2);
            nav.L2Pflag = Read_Field(line, 3);
            std::getline(file, line);
            nav.sACC = Read_Field(line, 0);
            nav.sHEA = Read_Field(line, 1);
            nav.TGD = Read_Field(line, 2);
            nav.IODC = Read_Field(line, 3);
            std::getline(file, line);
            nav.TTN = Read_Field(line, 0);
            nav.fit = Read_Field(line, 1);
            nav.spare1 = Read_Field(line, 2);
            nav.spare2 = Read_Field(line, 3);

            navs.push_back(nav); // 添加到列表中
            //std::cout<<nav.sPRN<<std::endl;
        }
        else
        {
            std::getline(file, line);
            std::getline(file, line);
            std::getline(file, line);
        }
        //std::cout<<navs.size()<<std::endl;

    }
    //std::cout << "读取了 " << navs.size() << " 条卫星星历数据\n";
}

void Positioning(Rinex_Navigation_Message_Data& data)
{
    std::string PRN_flag;
    //std::cout<<"开始位置解算";
    for(std::vector<Navi_Data_Body>::iterator it=data.vData_Body.begin(); it != data.vData_Body.end();)
    {
        PRN_flag=it->sPRN;//
        Rinex_Navigation_Message_Result result(PRN_flag);
        while(PRN_flag==it->sPRN)
        {

            if(it->sHEA!=0)
            {
                //如果非0 说明卫星不健康 则直接跳过该历元
                it++;
                //std::cout<<it->sPRN<<"\tHealth: "<<it->sHEA<<"\tTime"<<it->TOE<<std::endl;
                continue;
            }
//            if(it->sPRN[0]=='C'&&it->sPRN[1]=='0'&&it->sPRN[2]=='2')
//            {
//                BDS_Positioning(data, it, result);
//            }
            if(PRN_flag[0]=='G')
            {
                GPS_Positioning(data, it, result);
            }
            else if(PRN_flag[0]=='C')
            {
                BDS_Positioning(data, it, result);
            }

            //下一组数据
            it++;
        }
        data.vResult.push_back(result);
    }
    //调试输出
//    auto it= data.vData_Body.begin();
//    Rinex_Navigation_Message_Result result(PRN_flag);
//    GPS_Positioning(data,it, result);
}

void GPS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result)
{
    int deltat=-3600;//GPS广播星历间隔2小时
    int step=60;//解算步长
    while(deltat<3600)
    {


        double n0 = 0;//平均运动速率(未修正)
        double n = 0;//平均运动速率(修正)
        n0 = std::sqrt(data.GM) / it->sqrtA / it->sqrtA / it->sqrtA;
        n = n0 + it->deltan;

        double M_cur = it->M0 + n * deltat;//计算平近点角

        double E_cur, E_tmp;//偏近点角
//        E_cur = 2;
//        E_tmp = 1;
//        while (fabs(E_cur - E_tmp) > 1e-9) {
//            E_tmp = E_cur;
//            E_cur = sin(E_tmp) * it->e + M_cur;
//        }
        E_cur = M_cur;
        do {
            E_tmp = E_cur;
            E_cur = E_tmp - (E_tmp - it->e * sin(E_tmp) - M_cur) / (1 - it->e * cos(E_tmp));
        } while (fabs(E_cur - E_tmp) > 1e-12);

        //计算真近点角
        double f_cur = atan2(sqrt(1 - it->e * it->e) * sin(E_cur), cos(E_cur) - it->e);
        //计算升交距角（未经改正的）
        double u_cur_nocorr = it->omega + f_cur;
        //计算卫星向径（未经改正的）
        double r_cur_nocorr = it->sqrtA * it->sqrtA * (1 - cos(E_cur) * it->e);

        //计算摄动改正项
        double u_corr = it->Cus * sin(2 * u_cur_nocorr) + it->Cuc * cos(2 * u_cur_nocorr);
        double r_corr = it->Crs * sin(2 * u_cur_nocorr) + it->Crc * cos(2 * u_cur_nocorr);
        double i_corr = it->Cis * sin(2 * u_cur_nocorr) + it->Cic * cos(2 * u_cur_nocorr);

        //进行摄动改正
        double u_cur = u_cur_nocorr + u_corr;
        double r_cur = r_cur_nocorr + r_corr;
        double i_cur = it->i0 + it->IDOT * deltat + i_corr;

        //计算卫星在轨道平面坐标系（xoy）中的位置
        double x_cur = r_cur * cos(u_cur);
        double y_cur = r_cur * sin(u_cur);

        //计算观测瞬间升交点经度（大地经度=赤经减去GAST）
        //t时刻升交点赤经
        //double OMEGA_cur=it->OMEGA0+it->OMEGADOT*deltat;

        double omega_e = 7.292115e-5;//为地球自转速度
        //t时刻大地经度
        double Longitude_cur = it->OMEGA0 + it->OMEGADOT * deltat - (deltat + it->TOE) * omega_e;

        //卫星在瞬时地心地固坐标系下的坐标
        double X = x_cur * cos(Longitude_cur) - y_cur * cos(i_cur) * sin(Longitude_cur);
        double Y = x_cur * sin(Longitude_cur) + y_cur * cos(i_cur) * cos(Longitude_cur);
        double Z = y_cur * sin(i_cur);

        double xp = 0.0634 * 3.1415926/648000.0;
        double yp = 0.3316 * 3.1415926/648000.0;

        double X_Polar_corr = X + xp * Z;
        double Y_Polar_corr = Y - yp * Z;
        double Z_Polar_corr = - xp * X + yp * Y + Z;


        double refTime=0;//2019 12 01 00 00 00 周内秒为0
        double t_cur = deltat + it->TOE;
        double dt    = t_cur - refTime;
        double theta = omega_e * dt;

        // 把瞬时 ECEF 逆旋转到参考时刻 ECEF
        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);
        double X_ref =  cos_t * X_Polar_corr - sin_t * Y_Polar_corr;
        double Y_ref =  sin_t * X_Polar_corr + cos_t * Y_Polar_corr;
        double Z_ref =  Z_Polar_corr;

        Cartesian_XYZ_Coord coord_(X_ref,Y_ref,Z_ref);
        result.vXYZ_Coordinate.push_back(coord_);
        result.vTimeStamp.push_back(deltat+it->TOE);

//        double tmp = sqrt(X * X + Y * Y + Z * Z);
//        std::cout <<"GPS Week_Seconds:"<<deltat+it->TOE<<"\tPRN is "<< it->sPRN<<std::endl;
//        std::cout << X << '\t' << Y << '\t' << Z << '\t' << tmp << std::endl;
        deltat+=step;
    }
}

void BDS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result)
{
    int deltat=-1800;//BDS广播星历间隔1小时
    int step=60;//解算步长
    bool GEO_Flag=0;
    if(it->sPRN[1]=='0'&&it->sPRN[2]<'6'&&it->sPRN[2]>'0')
        GEO_Flag=1;
    while(deltat<1800)
    {

        double deltat_corr=deltat+it->sa0+it->sa1*deltat+it->sa2*deltat*deltat;


        double n0 = 0;//平均运动速率(未修正)
        double n = 0;//平均运动速率(修正)
        n0 = std::sqrt(data.GM) / it->sqrtA / it->sqrtA / it->sqrtA;
        n = n0 + it->deltan;

        double M_cur = it->M0 + n * deltat_corr;//计算平近点角

        double E_cur, E_tmp;//偏近点角
        E_cur = M_cur;
        do {
            E_tmp = E_cur;
            E_cur = E_tmp - (E_tmp - it->e * sin(E_tmp) - M_cur) / (1 - it->e * cos(E_tmp));
        } while (fabs(E_cur - E_tmp) > 1e-12);


        //计算真近点角
        double f_cur = atan2(sqrt(1 - it->e * it->e) * sin(E_cur), cos(E_cur) - it->e);

        //计算升交距角（未经改正的）
        double u_cur_nocorr = it->omega + f_cur;
        //计算卫星向径（未经改正的）
        double r_cur_nocorr = it->sqrtA * it->sqrtA * (1 - cos(E_cur) * it->e);

        //计算摄动改正项
        double u_corr = it->Cus * sin(2 * u_cur_nocorr) + it->Cuc * cos(2 * u_cur_nocorr);
        double r_corr = it->Crs * sin(2 * u_cur_nocorr) + it->Crc * cos(2 * u_cur_nocorr);
        double i_corr = it->Cis * sin(2 * u_cur_nocorr) + it->Cic * cos(2 * u_cur_nocorr);

        //进行摄动改正
        double u_cur = u_cur_nocorr + u_corr;
        double r_cur = r_cur_nocorr + r_corr;
        double i_cur = it->i0 + it->IDOT * deltat_corr + i_corr;


        //计算卫星在轨道平面坐标系（xoy）中的位置
        double x_cur = r_cur * cos(u_cur);
        double y_cur = r_cur * sin(u_cur);

        //计算观测瞬间升交点经度（大地经度=赤经减去GAST）
        //t时刻升交点赤经
        //double OMEGA_cur=it->OMEGA0+it->OMEGADOT*deltat;

        double omega_e = 7.2921151467e-5;//为地球自转速度
        //t时刻大地经度
        //double Longitude_cur = it->OMEGA0 + it->OMEGADOT * deltat - (deltat+it->TOE) * omega_e;
        double Longitude_cur = it->OMEGA0 + (it->OMEGADOT - omega_e) * deltat_corr - omega_e * (it->TOE);

        //GEO卫星单独处理
        if(GEO_Flag)
        {
            Longitude_cur = it->OMEGA0 + it->OMEGADOT * deltat_corr - omega_e * (it->TOE);
        }

        //卫星在瞬时地心地固坐标系下的坐标
        double X = x_cur * cos(Longitude_cur) - y_cur * cos(i_cur) * sin(Longitude_cur);
        double Y = x_cur * sin(Longitude_cur) + y_cur * cos(i_cur) * cos(Longitude_cur);
        double Z = y_cur * sin(i_cur);




        double xp = 0.0634 * 3.1415926/648000.0;
        double yp = 0.3316 * 3.1415926/648000.0;

        xp=0;
        yp=0;

        double X_Polar_corr = X + xp * Z;
        double Y_Polar_corr = Y - yp * Z;
        double Z_Polar_corr = - xp * X + yp * Y + Z;




        //GEO卫星单独处理
        if(GEO_Flag)
        {
            double f=-5*3.1415926/180;
            double X_GEO_corr,Y_GEO_corr,Z_GEO_corr;

            X_GEO_corr= X_Polar_corr;
            Y_GEO_corr= cos(f)*Y_Polar_corr+ sin(f)*Z_Polar_corr;
            Z_GEO_corr= -1.0*sin(f)*Y_Polar_corr+ cos(f)*Z_Polar_corr;

            X_Polar_corr=X_GEO_corr;
            Y_Polar_corr=Y_GEO_corr;
            Z_Polar_corr=Z_GEO_corr;
        }

        double refTime=0;//2019 12 01 00 00 00 周内秒为0
        double t_cur = deltat_corr + it->TOE;
        if(it->sPRN[0]=='C')
        {
            t_cur+=14;
        }
        double dt    = t_cur - refTime;
        double theta = omega_e * dt;

        // 把瞬时 ECEF 逆旋转到参考时刻 ECEF
        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);

        double X_ref =  cos_t * X_Polar_corr - sin_t * Y_Polar_corr;
        double Y_ref =  sin_t * X_Polar_corr + cos_t * Y_Polar_corr;
        double Z_ref =  Z_Polar_corr;


        //Cartesian_XYZ_Coord coord_(X,Y,Z);
        Cartesian_XYZ_Coord coord_(X_ref,Y_ref,Z_ref);
        result.vXYZ_Coordinate.push_back(coord_);
//        result.vTimeStamp.push_back(deltat_corr+it->TOE);
        result.vTimeStamp.push_back(t_cur);
//        double tmp = sqrt(X * X + Y * Y + Z * Z);
//        std::cout <<"GPS Week_Seconds:"<<deltat+it->TOE<<"\tPRN is "<< it->sPRN<<std::endl;
//        std::cout << X << '\t' << Y << '\t' << Z << '\t' << tmp << std::endl;
        deltat+=step;
    }
}

void Write2CSV(Rinex_Navigation_Message_Data &data)
{
    std::string filename="RINEX_GPS_BDS_Step60.csv";

    std::cout << "vResult size: " << data.vResult.size() << std::endl;

    std::ofstream file_(filename);
    if (!file_.is_open()) {
        std::cerr << "FILEOPEN ERROR: " << filename << std::endl;
        return;
    }

    file_ << "PRN号,X坐标,Y坐标,Z坐标,周内秒\n";

    for (auto it=data.vResult.begin();it!=data.vResult.end();it++)
    {
        auto it3=it->vTimeStamp.begin();
        for (auto it2=it->vXYZ_Coordinate.begin();it2!=it->vXYZ_Coordinate.end();it2++,it3++) {
            file_ << it->sPRN<< ","<< it2->X << ","<< it2->Y << ","<< it2->Z<< ","<< *it3 << "\n";
        }
    }
    file_.close();
    return;
}