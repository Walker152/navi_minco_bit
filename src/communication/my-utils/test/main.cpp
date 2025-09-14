#include "../MyUtils/libmyutils.hpp"
#include <iostream>
#include <queue>
#include <string>
#include <sys/signal.h>
using namespace std;

unsigned long long start_time;

void sigintHandler( int signo )
{
    MyUtils::MyTimer::TimerManager::stop( );
    cout << "catch sigint" << endl;
    cout << "总用时: "
         << double( MyUtils::getCurrentMillisecs( ) - start_time ) / 1000.0
         << "秒钟" << endl;
    return;
}

int main( )
{
    start_time = MyUtils::getCurrentMillisecs( );

    signal( SIGINT, sigintHandler );

    

    return 0;
}
