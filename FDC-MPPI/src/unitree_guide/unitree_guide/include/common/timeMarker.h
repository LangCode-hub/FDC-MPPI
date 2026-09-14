/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TIMEMARKER_H
#define TIMEMARKER_H

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

// Return a microsecond-resolution timestamp; requires <sys/time.h>.
inline long long getSystemTime(){
    struct timeval t;  
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;  
}
// Return a timestamp in seconds using getSystemTime().
inline double getTimeSecond(){
    double time = getSystemTime() * 0.000001;
    return time;
}
// Wait until waitTime microseconds have elapsed since startTime.
inline void absoluteWait(long long startTime, long long waitTime){
    if(getSystemTime() - startTime > waitTime){
        std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        << "The program has already cost " << getSystemTime() - startTime << "us." << std::endl;
    }
    while(getSystemTime() - startTime < waitTime){
        usleep(50);
    }
}

#endif //TIMEMARKER_H