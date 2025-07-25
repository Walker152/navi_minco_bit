#include "communication.hpp"


MyUtils::Net::FdManager Communication::fd_manager;
MyUtils::MyTimer::TimerManager Communication::timer_manager;

std::string Communication::STM32_PORT( "/dev/ttyACM0" );

// int count_time = 0;
 
void Communication::stm32_read_cb( ByteArray arr )
{
    static MyUtils::DataType::ByteArray stm32_recv_buffer;
    
stm32_recv_buffer.append( arr.get( ), arr.size( ) );                                                        

    char *temp = ( char * ) stm32_recv_buffer.get( );
    if ( stm32_recv_buffer.size( ) < 2 )
    {
        ROS_WARN("1");
        return;
    }
    bool flag = false;

    for ( int i = 0; i < stm32_recv_buffer.size( ) - 1; i++ )
    {
        if ( temp[ i ] == char( 0xa5 ) && temp[ i + 1 ] == char( 0x5a ) )
        {
            stm32_recv_buffer.sub( i, stm32_recv_buffer.size( ) -
                                          1 ); // START1 -> END
            flag = true;
            break;
        }
    }
    if ( !flag )

    {
        ROS_WARN("2");
        return;
    }
    

    if ( stm32_recv_buffer.size( ) < sizeof( PacketHeader ) )
    {
        ROS_WARN("3");
        return;
    }
    char *msg = ( char * ) stm32_recv_buffer.get( );
    PacketHeader *header = ( PacketHeader * ) msg;
    const int len = sizeof( PacketHeader ) + header->data_len; 
    // cout<<"----------------------sizeof packet header"<< sizeof(PacketHeader) << "-------------"<<endl;
    // cout<<"----------------------sizeof data len "<< (int)header->data_len << "-------------"<<endl;
    // cout<<"----------------------sizeof len "<< sizeof( PacketHeader ) + (int)header->data_len << "-------------"<<endl;

    // const int len  = 34;
    if ( stm32_recv_buffer.size( ) < len )
    {
        ROS_WARN("4");
        return;
    }
    // cout<<"THE PACKET MODE IS " << (int)header->packet_type << endl;
    char *info_rec = ( char * ) stm32_recv_buffer.get( );
    if (check(info_rec, len)) {// 校验
        // auto msg = packet_handler.handlePacket(info_rec, (int) header->packet_type);
        switch (header->packet_type) {
        // case ENUM_PACKET_SENTRY_ACCEPT_STATUS_DATA:
        //     // motor_speed_pub.publish();
        //     break;
        case ENUM_PACKET_NAV_DATA:{
            NavRes* nav_data = (NavRes*)(info_rec + sizeof(PacketHeader));
            nav_publish(nav_data);
            break;}
        default:{
            break;}
        };
      }
    
    else
    {
        std::cout << "checksum incorrect" << std::endl;
    }
    
    stm32_recv_buffer = stm32_recv_buffer.sub( len );
    return;
}



