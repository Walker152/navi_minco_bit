#include "../src/custom_protocol.hpp"
#include "MyUtils/DataType/ByteArray.hpp"
#include "MyUtils/Net/FdManager.hpp"
#include "MyUtils/Net/SerialPort.hpp"
#include "MyUtils/Thread/ThreadManager.hpp"
#include "MyUtils/Timer/Timer.hpp"
#include <iostream>
#include <thread>
using namespace std;
using MyUtils::DataType::ByteArray;

MyUtils::Net::FdManager fd_manager;
MyUtils::MyTimer::TimerManager timer_manager;

void emergency_read_cb( MyUtils::DataType::ByteArray arr, void * )
{
    // MyUtils::print( arr.get( ), arr.size( ) );
    if ( EmergencyResponse_on_Packet( ).check( arr.get( ), arr.size( ) ) )
    {
    }
    else if ( EmergencyResponse_off_Packet( ).check( arr.get( ), arr.size( ) ) )
    {
    }
}

void puncture_read_cb( MyUtils::DataType::ByteArray arr, void * )
{
    MyUtils::print( arr.get( ), arr.size( ) );
}

void arm_read_cb( MyUtils::DataType::ByteArray arr, void * )
{
    MyUtils::print( arr.get( ), arr.size( ) );
}

static void __open( const string &name, const string &port,
                    MyUtils::Net::fd_read_cb read_cb, void *read_cb_arg,
                    int _baud_rate = 9600, int _n_bits = 8,
                    int _stop_length = 1, char _check_type = 'N' )
{
    if ( fd_manager.exist( name ) )
        return;

    int fd = MyUtils::Net::SerialPort::open( port, _baud_rate, _n_bits,
                                             _stop_length, _check_type );
    cout << fd << endl;
    try
    {
        if ( fd != -1 )
            fd_manager.add( name, fd, read_cb, read_cb_arg );
    } catch ( const char *e )
    {
    }
}

int main( )
{
    timer_manager.addTimer( 2000, true, []( ) {
        __open( "arm", "/dev/pts/6", arm_read_cb, NULL );
    } );

    timer_manager.addTimer( 2000, true, []( ) {
        __open( "puncture", "/dev/pts/8", puncture_read_cb, NULL );
    } );

    timer_manager.addTimer( 2000, true, []( ) {
        __open( "emergency", "/dev/pts/10", emergency_read_cb, NULL );
    } );

    std::thread t( [ &fd_manager ]( ) { fd_manager.run( ); } );
    MyUtils::Thread::ThreadManager::getInstance( ).add( t.native_handle( ) );

    timer_manager.addTimer( 50, true, [ &fd_manager ]( ) {
        EmergencyRequestPacket p;
        char *data = ( char * ) &p;

        if ( fd_manager.send( "emergency", data, sizeof( p ) ) == 0 )
        {
            MyUtils::print( data, sizeof( p ) );
        }
        else
        {
        }
    } );

    timer_manager.run( );
    return 0;
}
