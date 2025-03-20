#pragma once
#include <chrono>
#include <string>
#include <iostream>

class timer
{
private:
    std::chrono::system_clock::time_point timeStart;//开始时间
    std::chrono::system_clock::time_point timeStop;//结束时间
    int64_t costTime{};
    
public:
    void start()
    {
        timeStart=std::chrono::system_clock::now();
    }
    void stop()
    {
        timeStop=std::chrono::system_clock::now();
        costTime=std::chrono::duration_cast<std::chrono::milliseconds>(timeStop - timeStart).count();
    }
    void print(const std::string& info,bool print_min=false) const
    {
        std::cout << info << costTime << "ms" << std::endl;
        if(print_min)
            std::cout << info << costTime/1000.0/60.0 << "min" << std::endl;
        //std::cout << info << costTime/1000.0/60.0 << "min" << std::endl;
    }
};


