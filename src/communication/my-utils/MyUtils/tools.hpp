#pragma once
#include "DataType/ByteArray.hpp"
#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <unistd.h>

namespace MyUtils
{
using namespace std;
using MyUtils::DataType::ByteArray;

class Tools
{

  private:
    Tools( const Tools & ) = delete;
    Tools( Tools && ) = delete;
    Tools( ) = delete;
    ~Tools( ) = delete;

  public:
    /**
     * @brief 读取所有内容到string中
     * @param [in] file_name 文件名
     * @param [out] 内容保存位置
     * @return 0 on successful and -1 on error
     */
    static int readAll( const string &file_name, ByteArray &arr )
    {
        std::ifstream t;
        t.open( file_name ); // open input file
        if ( t.is_open( ) == false )
        {
            printf( "%s打开失败", file_name.c_str( ) );
            return -1;
        }
        t.seekg( 0, std::ios::end ); // go to the end
        auto length = t.tellg( );    // report location (this is the length)
        t.seekg( 0, std::ios::beg ); // go back to the beginning
        arr = ByteArray( length );
        t.read( ( char * ) arr.get( ),
                length ); // read the whole file into the buffer
        t.close( );       // close file handle
        return 0;
    }

    /**
     * @brief 读取所有内容到string中
     * @param [in] file_name 文件名
     * @param [out] s 文件内容保存变量
     * @return 0 on successful and -1 on error
     */
    static int readAll( const string &file_name, string &s )
    {
        ifstream in;
        in.open( file_name, std::ios_base::in );
        if ( !in.is_open( ) )
        {
            printf( "%s打开失败", file_name.c_str( ) );
            return -1;
        }
        else
        {
            std::stringstream buffer;
            buffer << in.rdbuf( );
            s = buffer.str( );
            return 0;
        }
    }

    /**
     * @brief
     * 将16进制字符串转为十进制数字。要求字符串中必须满足正则表达式[a-fA-F0-9]+
     * @throw string
     * @return _T 转为十进制数字的类型
     */
    template < typename _T > static _T hex2dec( const std::string &s )
    {
        auto begin = s.begin( );
        _T ret = 0;
        while ( begin < s.end( ) )
        {
            ret <<= 4; // ret *= 16
            if ( *begin >= 'a' && *begin <= 'f' )
            {
                ret += *begin - 'a' + 10;
            }
            else if ( *begin >= 'A' && *begin <= 'F' )
            {
                ret += *begin - 'A' + 10;
            }
            else if ( *begin >= '0' && *begin <= '9' )
            {
                ret += *begin - '0';
            }
            else
            {
                char temp[ 1024 ];
                sprintf( temp, "\"%s\" 不满足[a-fA-F0-9]+的正则条件",
                         s.c_str( ) );
                throw( temp );
            }
            ++begin;
        }
        return ret;
    }

    /**
     * @brief 将字节流按照16进制打印
     * @param [in] data 字节流
     * @param [in] len 字节流长度
     */
    static void print( const void *data, const size_t len )
    {
        const char *d = ( const char * ) data;
        for ( size_t i = 0; i < len; i++ )
        {
            printf( "0x%02x ", d[ i ] & 0xff );
        }
        printf( "\n" );
    }

    /**
     * @brief
     * 该函数用于获取当前程序的路径和程序名称。如果processdir或者processname为NULL，则返回值为-1；
     * 否则返回值为程序名称字符串长度。
     * @param [out] processdir 保存当前程序路径
     * @param [out] processname 保存程序名称
     * @param [in] processdir 和 processname的缓冲区长度
     * @return -1 表示出错；否则返回值为程序名称字符串长度
     */
    static ssize_t getExecutablePath( char *processdir, char *processname, size_t len )
    {
        if ( processdir == NULL || processname == NULL )
        {
            return -1;
        }
        char *path_end;
        ssize_t _len = 0;
        if ( ( _len = readlink( "/proc/self/exe", processdir, len ) ) <= 0 )
            return -1;
        processdir[ _len ] = 0;
        // 或者最后一个'/'所在的位置
        path_end = strrchr( processdir, '/' );
        if ( path_end == NULL )
            return -1;
        ++path_end;

        strcpy( processname, path_end );
        *path_end = '\0';
        return ( ssize_t ) ( path_end - processdir );
    }

    /**
     * @brief
     * 关于程序中的计时问题的一些基本概念参阅：https://en.wikipedia.org/wiki/Elapsed_real_time
     * https://stackoverflow.com/questions/556405/what-do-real-user-and-sys-mean-in-the-output-of-time1
     * 这里实现的是计算实际的运行时间.返回值以毫秒为单位。但是值得注意的是，该程序的返回值以什么时间为基准计算的是
     * 不确定的。因此只能用来计算时间差。根据两次相减的结果可以得到wall time
     * @return 以毫秒为单位
     */
    static inline int64_t getCurrentMillisecs( )
    {
        // struct timespec ts;
        // clock_gettime( CLOCK_MONOTONIC, &ts );
        // return ts.tv_sec * 1000 + ts.tv_nsec / ( 1000 * 1000 );

        //这两段代码实现的效果是相同的，且第一段的代码在实际运行上要比下面的快，但是
        //运行速度差距几乎忽略不计，出于通用性的考虑，采用下面的代码

        return std::chrono::duration_cast< std::chrono::milliseconds >(
                   std::chrono::steady_clock::now( ).time_since_epoch( ) )
            .count( );
    }

    /**
     * @brief
     * 对字符串中的字符进行替换.将原字符串中的old_value全部替换成new_value，是在原字符串的基础之上进行替换，
     * 如果new_value中包含old_value的内容，并不会对new_value也进行替换。
     * 例如：string s = "12789", old_value = "78",
     * new_value="1278";则替换之后是"1212789";
     * @param old_value [in] 原字符串中要被替换的字符
     * @param new_value [in]
     */
    static inline void replaceSrc( const string &old_value, const string &new_value,
                            string &str )
    {
        string::size_type pos( 0 );
        while ( ( pos = str.find( old_value, pos ) ) != string::npos )
        {
            str.replace( pos, old_value.length( ), new_value );
            pos += old_value.length( );
        }
    }

    /**
     * @brief
     * 对字符串中的字符进行替换.将原字符串中的old_value全部替换成new_value，但不改变原字符串
     * 而是返回一个新的字符串.如果new_value中包含old_value的内容，并不会对new_value也进行替换。
     * 例如：string s = "12789", old_value = "78",
     * new_value="1278";则替换之后的返回值是"1212789";
     * @param old_value [in] 原字符串中要被替换的字符
     * @param new_value [in]
     */
    static inline string replace( const string &old_value, const string &new_value,
                           const string &str )
    {
        string ret = str;
        replaceSrc( old_value, new_value, ret );
        return ret;
    }

    /**
     * @brief 从fd中读取nbytes个字节到buf指向的地址空间
     * @param fd [in] 文件描述符,从文件描述符中读取数据
     * @return 返回值小于0说明出错；否则返回值为已经读取的字节数目
     */
    static ssize_t readn( int fd, char *buf, const size_t &nbytes )
    {
        size_t nleft;
        ssize_t nread;
        nleft = nbytes;
        while ( nleft > 0 )
        {
            if ( ( nread = ::read( fd, ( void * ) buf, nleft ) ) < 0 )
            {
                if ( nleft == nbytes )
                    return -1; // error, return -1
                else
                    break; // error, return amount read so far
            }
            else if ( nread == 0 )
            {
                break; // EOF
            }
            nleft -= nread;
            buf += nread;
        }
        return ( nbytes - nleft );
    }

    /**
     * @brief 向fd写入nbytes数据，从buf指向的地址空间取数据
     * @return 返回值小于0则失败，否则返回值是成功写入的字节数目
     */
    static ssize_t writen( int fd, const char *buf, const size_t &nbytes )
    {
        size_t nleft;
        ssize_t nwritten;
        nleft = nbytes;
        while ( nleft > 0 )
        {
            if ( ( nwritten = ::write( fd, ( void * ) buf, nleft ) ) < 0 )
            {
                if ( nleft == nbytes )
                    return -1; // error
                else
                    break; // error, return amount written so far
            }
            else if ( nwritten == 0 )
            {
                break;
            }
            nleft -= nwritten;
            buf += nwritten;
        }
        return ( nbytes - nleft );
    }

    /**
     * @param 打包函数
     */
    template < class F, class... Args >
    static inline auto warpFunction1( F &&f, Args &&...args )
        -> std::function< decltype( f( args... ) ) >
    {
        return bind( forward< F >( f ), forward< Args >( args )... );
    }

    /**
     * @param 这里将函数打包成void()类型的无参回调函数
     */
    template < class F, class... Args >
    static inline std::function< void( ) > warpFunction2( F &&f, Args &&...args )
    {
        auto task =
            bind( forward< F >( f ),
                  forward< Args >( args )... ); // 把函数入口及参数,打包(绑定)

        std::function< void( ) > cb = [ task ]( ) { task( ); };
        return cb;
    }
};
} // namespace MyUtils