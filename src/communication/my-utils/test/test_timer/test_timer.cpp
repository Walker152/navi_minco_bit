#include "../MyUtils/libmyutils.hpp"
#include <iostream>
#include <string>
#include <sys/signal.h>
using namespace std;

unsigned long long start_time;

void fifthTimerFunctionTest( unsigned long long start_time )
{
    cout << "第五个定时器, 执行时与开始时间相差: "
         << ( MyUtils::getCurrentMillisecs( ) - start_time ) << "毫秒" << endl;
}

int main( )
{
    start_time = MyUtils::getCurrentMillisecs( );
    MyUtils::MyTimer::TimerManager timer_manager;

    {
        for ( long i = 0; i < 10000000; i++ )
        {
            MyUtils::getCurrentMillisecs( );
        }
    }
    {
        auto start_time = MyUtils::getCurrentMillisecs( );
        timer_manager.addTimer( 2001, false, [ = ]( ) {
            cout << "第一个定时器, 执行时与开始时间相差: "
                 << ( MyUtils::getCurrentMillisecs( ) - start_time ) << "毫秒"
                 << endl;
        } );

        start_time = MyUtils::getCurrentMillisecs( );
        timer_manager.addTimer( 1001, false, [ = ]( ) {
            cout << "第二个定时器, 执行时与开始时间相差: "
                 << ( MyUtils::getCurrentMillisecs( ) - start_time ) << "毫秒"
                 << endl;
        } );

        start_time = MyUtils::getCurrentMillisecs( );
        timer_manager.addTimer( 4001, false, [ = ]( ) {
            cout << "第三个定时器, 执行时与开始时间相差: "
                 << ( MyUtils::getCurrentMillisecs( ) - start_time ) << "毫秒"
                 << endl;
        } );

        //测试在运行前后添加定时器是否有影响
        thread timer_manager_thread( [ & ]( ) { timer_manager.run( ); } );

        start_time = MyUtils::getCurrentMillisecs( );
        auto timer4 = timer_manager.addTimer( 3001, false, [ = ]( ) {
            cout << "第四个定时器, 执行时与开始时间相差: "
                 << ( MyUtils::getCurrentMillisecs( ) - start_time ) << "毫秒"
                 << endl;
        } );



        timer_manager.addTimer(
            1500, false, fifthTimerFunctionTest,
            MyUtils::getCurrentMillisecs( ) );

        //测试循环定时器
        auto timer6 = timer_manager.addTimer(
            500, true, [ = ]( ) {
                cout << "第六个定时器,循环定时器,执行时与开始时间相差: "
                     << ( MyUtils::getCurrentMillisecs( ) - start_time )
                     << "毫秒" << endl;
            } );

        timer_manager.addTimer(
            2000, false, [ = ]( ) {
                cout << "第七个定时器,将第六个计时器和第四个计时器删除"
                     << ( MyUtils::getCurrentMillisecs( ) - start_time )
                     << "毫秒" << endl;

                // 目前不允许在定时器内部调用定时器管理，在互锁问题没有解决之前

                // TODO 解决互锁问题
                // MyUtils::MyTimer::TimerManager::getInstance().delTimer(timer4);
                // MyUtils::MyTimer::TimerManager::getInstance().delTimer(timer6);
                // cout << "删除成功" << endl;
            } );

        std::this_thread::sleep_for( std::chrono::seconds( 2 ) );
        timer_manager.delTimer( timer6 );

        timer_manager_thread.join( );
    }
    return 0;
}
