#include "../MyUtils/libmyutils.hpp"

#include <iostream>
#include <thread>
using namespace std;

int main( )
{
    MyUtils::GlogHelper helper;
    std::thread timer_thread(
        []( ) { MyUtils::MyTimer::TimerManager::getInstance( ).run( ); } );

    /****************client*****************/
    MyUtils::Net::FdManager fd_manager;

    MyUtils::MyTimer::TimerManager::getInstance( ).addTimer(
        1000, true, [ & ]( ) {
            int fd = ( MyUtils::Net::Client::connect( SOCK_STREAM, 9999,
                                                      "127.0.0.1" ) );
            if ( fd == -1 )
            {
                return;
            }
            bufferevent_data_cb readcb = []( bufferevent *bev, void *ctx ) {
                char msg[ 1024 ];
                size_t len = bufferevent_read( bev, msg, sizeof( msg ) );
                msg[ len ] = '\0';
                printf( "recv %s from server\n", msg );
            };
            bufferevent *bev = fd_manager.add( fd, readcb );
            char s[ 1024 ];
            sprintf( s, "I'm from client %d", fd );
            write( fd, s, strlen( s ) );
            write( fd, s, strlen( s ) );
            write( fd, s, strlen( s ) );

            int ret = bufferevent_write( bev, ( void * ) s, strlen( s ) );
            ret = bufferevent_write( bev, "Hello", 5 );

            ret = bufferevent_write_buffer( bev, bev->output );

            // printf( s );
            // ::close( fd );
        } );

    /*******************FdManager*****************/
    std::thread fd_thread( [ & ]( ) { fd_manager.run( ); } );

    fd_thread.join( );
    timer_thread.join( );
    return 0;
}