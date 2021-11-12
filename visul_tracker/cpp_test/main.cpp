#include <iostream>
union send_msg
{
    int x; // 32 bit
    unsigned char data[2]; // 16 bit
}blob_pt_x, laser_pt;
struct send_msg2
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
}blob_pt_x2;


int main() {
    blob_pt_x2.u.data32 = 92;

    std::cout<<blob_pt_x2.u.data16[0]<<std::endl;
    return 0;
}
