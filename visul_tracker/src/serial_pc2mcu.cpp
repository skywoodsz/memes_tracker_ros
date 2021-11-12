//
// Created by skywoodsz on 2021/11/6.
//

#include "visul_tracker/serial_pc2mcu.h"

using namespace std;
using namespace boost::asio;

Serial_PC2MCU::Serial_PC2MCU()
{
    sp = new serial_port(iosev);
}

Serial_PC2MCU::~Serial_PC2MCU()
{
    if(sp)
    {
        delete sp;
    }
}

bool Serial_PC2MCU::init(const string port, const int baud)
{
    if (!sp)
    {
        std::cout<<"serial_port add fail!"<<std::endl;
        return false;
    }
    sp->open(port, ec); // open the port

    sp->set_option(serial_port::baud_rate(baud), ec); //波特率
    sp->set_option(serial_port::flow_control(serial_port::flow_control::none), ec);
    sp->set_option(serial_port::parity(serial_port::parity::none), ec);//无奇偶校验位
    sp->set_option(serial_port::stop_bits(serial_port::stop_bits::one), ec);//一个停止位
    sp->set_option(serial_port::character_size(8), ec);//字长

    std::cout<<"serial_port add good!"<<std::endl;
    return true;
}

bool Serial_PC2MCU::send_data(int bx, int by)
{
    unsigned char buf[7] = { 0 };
    blob_pt.u.data32 = bx;
    blob_pt.v.data32 = by;

    // laser_pt.u.data32 = lx;
    // laser_pt.v.data32 = ly;

    //报头
    for (int i = 0; i < 2; i++)
    {
        buf[i] = header[i];
    }
    //数据从高到低
    int j = 1;
    for (int i = 0; i < 2; i++)
    {
        buf[i + 2] = blob_pt.u.data16[j];
        buf[i + 4] = blob_pt.v.data16[j];
        // buf[i + 10] = laser_pt.u.data16[j];
        // buf[i + 14] = laser_pt.v.data16[j];
        j--;
    }
    //报尾
    buf[6] = ender[0];
    write(*sp,buffer(buf, 19), ec);
    return true;
}
