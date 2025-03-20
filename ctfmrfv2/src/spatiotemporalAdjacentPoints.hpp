#include <cmath>
/*
日期：2022年4月28日
作者：weixinhum
须知：单位是m
说明：时空临近点地面分割算法
*/
#pragma once

class SpatioTemporalAdjacentPoints
{
public:
    PTCloud *mPTCloud = nullptr; // 点云对象

    explicit SpatioTemporalAdjacentPoints(PTCloud *getPTCloud)
    {
        mPTCloud = getPTCloud;
    }

    // 点的平面间距计算
    static inline double ptDistance(float &x1, float &x2, float &y1, float &y2)
    {
        return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        // return abs(x2 - x1) + abs(y2 - y1);
    }

    // 时空临近障碍物搜索算法
    long run()
    {
        auto start = std::chrono::system_clock::now();
        int selline=1;
        // 这里改为从下往上搜应该能快不少
        for (int l = mPTCloud->rows - 1; l >= selline; l-- ) // 不考虑前面8根线，对于其他数据集的话应该要调整一下
        {
            if(mPTCloud->rows==64 && l<64-24)
                selline=2;
            if(mPTCloud->rows==32)
                selline=2;

            int selZoo = 3; // 搜索范围
            for (int p = 0; p < mPTCloud->cols; p++)
            {
                if (mPTCloud->mPTCloudInfo[l][p].mergeMarkType == 0)
                {
                    mPTCloud->mPTCloudInfo[l][p].setType = 0;
                    continue;
                }
                if (mPTCloud->mPTCloudInfo[l][p].setType == 20) // 已经是障碍物就不用再求一次
                {
                    continue;
                }
                if (mPTCloud->rows == 64)
                {
                    if (p > 850 && p < 950) // 帧间的交界处可能会有偏差，因此不适用于该算法
                    {
                        continue;
                    }
                }
                for (int sp = p - selZoo; sp <= p + selZoo; sp++)
                {
                    if (sp < 0 || sp >= mPTCloud->cols) // 越界则不搜索
                    {
                        continue;
                    }
                    if (mPTCloud->mPTCloudInfo[l - selline][sp].mergeMarkType == 0) // 搜索点无意义不搜索
                    {
                        continue;
                    }
                    float thrdist=0.03;
                    if(mPTCloud->rows == 32)
                        thrdist=-0.1;
                    if (mPTCloud->mPTCloudInfo[l][p].distance < mPTCloud->mPTCloudInfo[l - selline][sp].distance - thrdist)
                    {
                        mPTCloud->mPTCloudInfo[l][p].setType = 20;
                        break;
                    }
                }
            }
        }
        auto end = std::chrono::system_clock::now();
        long time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        return time;
    }

    ~SpatioTemporalAdjacentPoints() // 释放对象
    {
    }
};
