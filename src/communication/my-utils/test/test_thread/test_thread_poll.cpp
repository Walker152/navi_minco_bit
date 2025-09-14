#include "../../MyUtils/GLogHelper.hpp"
#include "../../MyUtils/Thread/ThreadPoll.hpp"
#include <chrono>
#include <iostream>
#include <random>
using namespace std;

int fun( int i ) {}

int main( )
{
    MyUtils::GlogHelper glog;
    MyUtils::Thread::ThreadPool pool( 100 );

    std::vector< std::future< int > > v;

    for ( int i = 0; i < 200; i++ )
    {
        v.push_back(pool.commit([i]()->int{
            std::random_device rd;
            std::default_random_engine e( rd( ) );
            std::uniform_int_distribution<> u( 1, 5 );
            int x = u( e );
            std::this_thread::sleep_for( std::chrono::seconds( u( e ) ) );
            std::thread::id id = std::this_thread::get_id( );
            printf( "i = %d, sleep_time = %d\n", i, x );
            return 0;
        }));
        // v.push_back( r );
    }

    cout << "添加完毕" << endl;

    for ( auto &&i : v )
    {
        i.get( );
    }

    // long result = 0;
    // pool.commit( [ & ]( ) {
    //     for ( int i = 0; i < 1e9; i++ )
    //     {
    //         result += i;
    //     }
    //     cout << "当前线程ID：" << std::this_thread::get_id( )
    //          << "\tresult = " << result << endl;
    // } );

    // auto r = pool.commit( [ & ]( ) -> bool {
    //     while ( result < 1e6 )
    //     {
    //         std::this_thread::yield( );
    //     }
    //     cout << "当前线程ID：" << std::this_thread::get_id( )
    //          << "\tresult = " << result << endl;
    //     return true;
    // } );

    // cout << "********************" << endl;
    // cout << r.get( ) << endl;
    // cout << "///////////////////////////" << endl;

    getc( stdin );

    return 0;
}