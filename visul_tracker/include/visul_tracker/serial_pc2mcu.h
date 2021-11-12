//
// Created by skywoodsz on 2021/11/6.
//

#ifndef CPP_SRC_SERIAL_PC2MCU_H
#define CPP_SRC_SERIAL_PC2MCU_H

#include<iostream>
#include<string>
#include<boost/asio.hpp>
#include<boost/bind.hpp>
using namespace std;
using namespace boost::asio;

class Serial_PC2MCU
{
public:
    Serial_PC2MCU();
    ~Serial_PC2MCU();
    /*
     * Function: port 初始化
     * Input: port: port name;
     *        baud: baudrate 115200, 9600 ...
     */
    bool init(const string port, const int baud);

    /*
     * Input: bx: blob_pt_x; by: blob_pt_y;
     *        lx: laser_pt_x; by: laser_pt_y;
     * Data form: 0x55,0x5A,| _, _, _, _,| _, _, _, _,| _, _, _, _,| _, _, _, _,| 0x0A;
     *                         blob_pt_x    blob_pt_y    laser_pt_x   laser_pt_y
     * */
    bool send_data(int bx, int by); //发送data

public:
    const unsigned char header[2] = {0x55,0x5A}; //报头
    const unsigned char ender[1] = {0xAA};	//报尾

    // data send form; union use the same memory
    struct send_msg
    {
        union U
        {
            int data32;
            unsigned char data16[2];
        }u;

        union V
        {
            int data32;
            unsigned char data16[2];
        }v;
    }blob_pt, laser_pt;

    // port
    io_service iosev;
    serial_port *sp;
    boost::system::error_code ec;
};



#endif //CPP_SRC_SERIAL_PC2MCU_H
