#include "../MyUtils/libmyutils.hpp"

int main( )
{
    MyUtils::GlogHelper helper;

    MyUtils::SP::SerialPort serial_port1( "/dev/pts/1" );
    MyUtils::SP::SerialPort serial_port2( "/dev/pts/3" );

    int fd1 = serial_port1.open( );
    int fd2 = serial_port2.open( );

    MyUtils::Net::FdManager fd_manager;
    std::thread fd_manager_thread( [ & ]( ) { fd_manager.run( ); } );
    auto readcb = []( bufferevent *bev, void *ctx ) {
        char msg[ 1024 ];
        size_t len = bufferevent_read( bev, msg, sizeof( msg ) );
        msg[ len ] = '\0';
    };
    auto bev1 = fd_manager.add( fd1, readcb );
    auto bev2 = fd_manager.add( fd2, readcb );

    fd_manager_thread.join();

    return 0;
}