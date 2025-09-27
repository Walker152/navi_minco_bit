#pragma once
#include "../tools.hpp"

#include <atomic>
#include <errno.h>  //与全局变量 errno 相关的定义
#include <fcntl.h>  //文件控制定义
#include <getopt.h> //处理命令行参数
#include <stdio.h>  //标准输入输出,如printf、scanf以及文件操作
#include <stdlib.h> //标准库头文件，定义了五种类型、一些宏和通用工具函数
#include <string.h> //字符串操作
#include <string>
#include <sys/stat.h>  //文件状态
#include <sys/types.h> //定义数据类型，如 ssiz e_t off_t 等
#include <termios.h>   //终端I/O
#include <time.h>      //时间
#include <unistd.h>    //定义 read write close lseek 等Unix标准函数
namespace MyUtils
{

namespace Net
{
using namespace std;

class SerialPort
{
  public:
    /**
     * @brief 开启串口
     * @param _port [in] 端口文件路径
     * @param _baud_rate [in] 波特率
     * @param _n_bits [in] 数据位位数
     * @param _stop_length [in] 停止位，一般为1
     * @param _check_type [in] 校验模式, 'o'表示奇校验，'e'表示偶校验, 'N'
     * 表示无校验
     * @return 返回值是文件描述符fd。如果开启串口失败返回值为-1
     */
    static int open( const string &_port, int _baud_rate = 9600,
                     int _n_bits = 8, int _stop_length = 1,
                     char _check_type = 'N' ) noexcept
    {

        int fd = __openPort( _port );
        if ( -1 != fd )
        {
            if ( __config( fd, _baud_rate, _n_bits, _stop_length,
                           _check_type ) == 0 )
            {
            }
            else
            {
                ::close( fd );
                fd = -1;
            }
        }
        return fd;
    }

  private:
    static int __openPort( const string &_port ) noexcept
    {
        /*在打开串口时，除了需要用到 O_RDWR (可读写)选项标志外，
         * O_NOCTTY：告诉 Linux
         * “本程序不作为串口的‘控制终端’”。如果不使用该选项，会有一些输入字符影响进程运行（如一些产生中断信号的键盘输入字符等）。
         * O_NONBLOCK：标志则是告诉Linux,这个程序并不关心DCD信号线的状态——也就是不关心端口另一端是否已经连接。
         * 这里需要设置为非阻塞模式
         */
        int fd = ::open( _port.c_str( ), O_RDWR | O_NOCTTY | O_NONBLOCK );

        if ( -1 == fd )
        {
            perror( "open" );
            return -1;
        }

        tcflush( fd, TCIOFLUSH ); //清空输入输出缓存

        return fd;
    }

    /**
     * @brief: 配置端口设置
     * @return: 0 on success, -1 on error
     */
    static int __config( int fd, int _baud_rate, int _n_bits, int _stop_length,
                         char _check_type ) noexcept
    {
        struct termios newtio;

        bzero( &newtio, sizeof( struct termios ) );
        newtio.c_cflag |= CLOCAL | CREAD;
        newtio.c_cflag &= ~CSIZE;
        switch ( _n_bits )
        {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            return -1;
        }
        switch ( _check_type )
        {
        case 'o':
        case 'O': //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= ( INPCK | ISTRIP );
            break;
        case 'e':
        case 'E': //偶校验
            newtio.c_iflag |= ( INPCK | ISTRIP );
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N': //无校验
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            return -1;
        }

        // 设置停止位
        switch ( _stop_length )
        {
        case 1:
            newtio.c_cflag &= ~CSTOPB;
            break;
        case 2:
            newtio.c_cflag |= CSTOPB;
            break;
        default:
            return -1;
        }

        // 设置波特率 2400/4800/9600/19200/38400/57600/115200/230400
        switch ( _baud_rate )
        {
        case 2400:
            cfsetispeed( &newtio, B2400 );
            cfsetospeed( &newtio, B2400 );
            break;
        case 4800:
            cfsetispeed( &newtio, B4800 );
            cfsetospeed( &newtio, B4800 );
            break;
        case 9600:
            cfsetispeed( &newtio, B9600 );
            cfsetospeed( &newtio, B9600 );
            break;
        case 19200:
            cfsetispeed( &newtio, B19200 );
            cfsetospeed( &newtio, B19200 );
            break;
        case 38400:
            cfsetispeed( &newtio, B38400 );
            cfsetospeed( &newtio, B38400 );
            break;
        case 57600:
            cfsetispeed( &newtio, B57600 );
            cfsetospeed( &newtio, B57600 );
            break;
        case 115200:
            cfsetispeed( &newtio, B115200 );
            cfsetospeed( &newtio, B115200 );
            break;
        case 230400:
            cfsetispeed( &newtio, B230400 );
            cfsetospeed( &newtio, B230400 );
            break;
        default:
            cfsetispeed( &newtio, B9600 );
            cfsetospeed( &newtio, B9600 );
            break;
        }
        // 设置最少字符和等待时间。在对接收字符和等待时间没有特别要求的情况下，
        // 这里设置为了至少读入一个字符才会read返回，否则阻塞
        // newtio.c_cc[ VTIME ] = 0; // 读取一个字符等待1*(1/10)s
        // newtio.c_cc[ VMIN ] = 2 + packet_head_size * 2 + 1;  //
        // 读取字符的最少个数为1

        if ( ( tcsetattr( fd, TCSANOW, &newtio ) ) != 0 )
        {
            perror( "tcsetattr" );
            return -1;
        }
        tcflush( fd, TCIFLUSH );

        return 0;
    }
};

} // namespace Net
} // namespace MyUtils