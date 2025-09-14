#include "../../MyUtils/GLogHelper.hpp"
#include "../../MyUtils/Thread/ThreadPoll.hpp"
#include "../../MyUtils/Thread/ThreadSafeContainer.hpp"
#include "../../MyUtils/Timer/ProcessTimer.hpp"
#include <chrono>
#include <iostream>
#include <random>
using namespace MyUtils::Thread;
using namespace std;

// typedef ThreadSafeQueue< int > test_type;
// typedef ThreadSafeStack< int > test_type;
typedef ThreadSafeHeap< int > test_type;

void test1( )
{
    const int num_test = 10000;
    std::atomic< double > efficiency( 0 );
    {
        test_type q;
        MyUtils::Thread::ThreadPool pool( 1001 );

        for ( int i = 0; i < num_test; i++ )
        {
            pool.commit( [ i, &q, &efficiency ]( ) {
                std::random_device rd;
                std::default_random_engine e( rd( ) );
                std::uniform_int_distribution<> u( 1, 1000 );
                int x = u( e );
                MyUtils::MyTimer::ProcessTimer t1;
                std::this_thread::sleep_for(
                    std::chrono::milliseconds( u( e ) ) );
                if ( x & 1 )
                {
                    for ( int i = 0; i < ( x % 20 ); i++ )
                    {
                        q.get( );
                        q.pop( );
                    }
                }
                else
                {
                    for ( int i = 0; i < ( x % 100 ); i++ )
                        q.put( i + x );
                }

                // printf(
                //     "i = %d, sleep_time = %d, running time = %lld, 效率 =
                //     %f\n", i, x, t1.duration( ), double( x ) / double(
                //     t1.duration( ) ) );

                auto temp = double( x ) / double( t1.duration( ) );
                auto result = efficiency.load( ) + temp;
                efficiency.store( result );
            } );
        }
    } //线程池析构时会自动join
    std::cout << "平均效率" << efficiency / num_test << std::endl;
}

void test2( )
{
    { //测试共用内存
        test_type q;
        q.put( 10 );

        test_type p = q;

        p.put( 11 );
        LOG_ASSERT( q.size( ) == 2 ); // if true no log

        while ( p.size( ) )
        {
            p.pop( );
        }
        LOG_ASSERT( p.size( ) == q.size( ) ); // if true no log
    }

    {
        test_type q;
        q.put( 10 );

        test_type p = q;

        p.put( 11 );
        LOG_ASSERT( q.size( ) == 2 ); // if true no log

        for ( int i = 0; i < p.size( ); i++ )
        {
            p.get( );
        }
        LOG_ASSERT( p.size( ) == q.size( ) ); // if true no log
        LOG_ASSERT( p.size( ) == 2 );         // if true no log
    }

    {
        test_type q;
        q.put( 10 );
        test_type p = q;
        p.put( 123 );

        p = test_type( );
        LOG_ASSERT( p.size( ) != q.size( ) ); // if true no log
        LOG_ASSERT( p.size( ) == 0 );
    }

    {
        test_type q;
        q.put( 10 );
        test_type p = q;
        p.put( 123 );
        test_type t;
        t.copy( q );
        LOG_ASSERT( t.size( ) == q.size( ) );
        t.put( 123 );
        LOG_ASSERT( t.size( ) != p.size( ) );
        LOG_ASSERT( q.size( ) == p.size( ) );
    }
}

void test3( )
{
    test_type q;
    try
    {
        q.try_get( );
    } catch ( const int i )
    {
        //报错因为没有数据
    }

    q.put( 3 );
    try
    {
        q.try_get( );
    } catch ( const int i )
    {
    }
    LOG_ASSERT( q.size( ) == 1 );

    MyUtils::MyTimer::ProcessTimer t;
    try
    {
        q.try_pop( 1000 );
    } catch ( const int i )
    {
    }
    t.reset( );
    try
    {
        q.try_pop( 1000 );
    } catch ( const int i )
    {
    }
}

int main( )
{
    MyUtils::GlogHelper glog;
    MyUtils::MyTimer::ProcessTimer t;
    test1( );
    std::cout << "test1用时" << t.duration( ) << "毫秒"
              << std::endl; // 5096毫秒.

    test2( );
    test3( );
    return 0;
}