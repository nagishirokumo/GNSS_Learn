#include<stdio.h>
#include<windows.h>

#define BUFFER_SIZE 4096

#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)


void saveBinaryData(SOCKET& sock, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary | std::ios::app);
    if (!outFile) {
        std::cerr << "file open failed: " << filename << std::endl;
        return;
    }

    char buffer[BUFFER_SIZE] = {0};

    while (true) {
        int bytes_Received = recv(sock, buffer, sizeof(buffer), 0);
        if (bytes_Received > 0) {
            outFile.write(buffer, bytes_Received); // 直接写入二进制数据
            outFile.flush(); // 立即写入文件
        } else if (bytes_Received == 0) {
            std::cout << "连接已关闭" << std::endl;
            break;
        } else {
            std::cerr << "接收数据失败: " << WSAGetLastError() << std::endl;
            break;
        }
    }
    outFile.close(); // 关闭文件
}

bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port)
{
	WSADATA wsaData;
	SOCKADDR_IN addrSrv;

	if(!WSAStartup(MAKEWORD(1, 1), &wsaData))
	{
		if( (sock = socket(AF_INET, SOCK_STREAM ,0)) != INVALID_SOCKET )
		{
			addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
			addrSrv.sin_family = AF_INET;
			addrSrv.sin_port = htons(Port);
			connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));	
			return true;
		}
	}
	return false;
}

void CloseSocket(SOCKET& sock)
{
	closesocket(sock);
	WSACleanup();
};
