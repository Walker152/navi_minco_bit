#include "../MyUtils/libmyutils.hpp"

#include <iostream>
#include <thread>
using namespace std;

static void _socket_read_cb( bufferevent *bev, void *arg )
{
    char msg[ 4096 ];
    size_t len = bufferevent_read( bev, msg, sizeof( msg ) - 1 );
    msg[ len ] = '\0';
    char reply[] = "I has read your data";
    int ret = bufferevent_write( bev, reply, strlen( reply ) );
    return;
}

static void _socket_event_cb( bufferevent *bev, short events, void *arg )
{
    if ( events & BEV_EVENT_EOF )
    {
    }
    else if ( events & BEV_EVENT_ERROR )
    {
    }
    //这将自动close套接字和free读写缓冲区
    bufferevent_free( bev );
}


int main( )
{
    MyUtils::GlogHelper helper;

    MyUtils::Net::Server server(
        9999, 10,
        []( evconnlistener *listener, evutil_socket_t fd, struct sockaddr *sock,
            int socklen, void *arg ) {
            printf( "accept a client %d\n", fd );
            event_base *base = ( event_base * ) arg;
            //为这个客户端分配一个bufferevent
            bufferevent *bev = bufferevent_socket_new(
                base, fd, BEV_OPT_CLOSE_ON_FREE | BEV_OPT_THREADSAFE );
            bufferevent_setcb( bev, _socket_read_cb, NULL, _socket_event_cb,
                               NULL );
            bufferevent_enable( bev, EV_READ | EV_PERSIST );
        } );

    std::thread server_thread( [ & ]( ) { server.run( ); } );


    server_thread.join( );

    return 0;
}