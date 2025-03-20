/********************************
作者：黄伟鑫
日期：20220115
功能：点云地面分割结果评估
********************************/
#pragma once

#include <chrono>
#include <string>
#include "PTCloudDataFrame.hpp"

class PTCloudGSegEvaluation
{
public:
    PTCloud *mPTCloud = nullptr;                     // 点云对象
    std::chrono::system_clock::time_point timeStart; // 开始时间
    std::chrono::system_clock::time_point timeStop;  // 结束时间
    int64_t costTime{};
    unsigned int seq = 0;      // 当前计算帧所在系列
    unsigned int inSeqNum = 0; // 当前计算帧所在系列位置

    int confusionMatrix[850][2][2] = {0};        // 混淆矩阵
    int ptcount[850] = {0};                      // 混淆矩阵中的点数计算
    
    int mergeMarkType2confusionMatrix[30] = {0}; // 标注数值到混淆矩阵的转换
    int setType2confusionMatrix[30] = {0};       // 算法标记数值到混淆矩阵的转换

    
    double mPrecision[850]={0};//精确度
    double mRecall[850] = {0};//召回率
    double mF1[850] = {0};//F1
    double mAccuracy[850] = {0};//准确度
    double mIou[850] = {0};//交并比

    double mPrecision_nuscenes[10]={0};//精确度
    double mRecall_nuscenes[10] = {0};//召回率
    double mF1_nuscenes[10] = {0};//F1
    double mAccuracy_nuscenes[10] = {0};//准确度
    double mIou_nuscenes[10] = {0};//交并比
    double mcosttime_nuscenes[10] = {0};//花费时间

    double mcosttime[850] = {0};
    long long segCostTime[850] = {0};            // 用以记录每帧数据的耗时

    int sceneslen[2];
    float scenesprocesstime[2][850];

    explicit PTCloudGSegEvaluation(PTCloud *getPTCloud)
    {
        sceneslen[0] = 650;
        sceneslen[1] = 498;

        mPTCloud = getPTCloud;
        init();
        for (int i = 0; i < 850; i++)
        {
            // 初始化消耗时间计数
            segCostTime[i] = 0;
            mcosttime[i] = 0;
            // 初始化几个相关的评估指标
            mPrecision[i] = 0;
            mRecall[i] = 0;
            mF1[i] = 0;
            mAccuracy[i] = 0;
            mIou[i] = 0;
        }
        mergeMarkType2confusionMatrix[10] = 0; // 10->0,地面点
        mergeMarkType2confusionMatrix[20] = 1; // 20->1,非地面点

        setType2confusionMatrix[10] = 0; // 10->0,地面点
        setType2confusionMatrix[20] = 1; // 20->1,非地面点
    }

    void init()
    {
        for (size_t i = 0; i < 850; i++)
        {
            // 初始化混淆矩阵计数
            ptcount[i] = 0;
            // 初始化混淆矩阵
            for (int r = 0; r < 2; r++)
                for (int c = 0; c < 2; c++)
                {
                    confusionMatrix[i][r][c] = 0;
                }
        }
    }

    void start(int getSeq, int getSeqPTCloudNum)
    {
        // std::cout<<std::endl<<"Seq:"<<getSeq<<"    getSeqPTCloudNum:"<<getSeqPTCloudNum<<std::endl;
        seq = getSeq;
        inSeqNum = getSeqPTCloudNum;
        timeStart = std::chrono::system_clock::now();
    }

    void stop()
    {
        timeStop = std::chrono::system_clock::now();
        costTime = std::chrono::duration_cast<std::chrono::milliseconds>(timeStop - timeStart).count();
        segCostTime[seq] += costTime; // 获得地面分割的时间
        ptcount[seq]++;
        if (mPTCloud->rows == 128)
        {
            scenesprocesstime[seq][inSeqNum] = costTime;
        }
        // 更新混淆矩阵
        for (int l = 0; l < mPTCloud->rows; l++)
            for (int c = 0; c < mPTCloud->cols; c++)
            {
#if evaluations_40_8
                //evaluations within 8 m of the vehicle’s horizontal coordinate and a total distance between 40 and 50 m
                if(mPTCloud->mPTCloudInfo[l][c].distance>40 && abs(mPTCloud->mPTCloud[l][c].y)<8)
#endif
                {
                    if (mPTCloud->mPTCloudInfo[l][c].mergeMarkType != 0 && mPTCloud->mPTCloudInfo[l][c].mergeMarkType != 30)
                    {
                        int mconfusionM = mergeMarkType2confusionMatrix[mPTCloud->mPTCloudInfo[l][c].mergeMarkType];//0是地面点，1是非地面点
                        int sconfusionM = setType2confusionMatrix[mPTCloud->mPTCloudInfo[l][c].setType];//0是地面点，1是非地面点
                        confusionMatrix[seq][mconfusionM][sconfusionM]++;
                    }
                }
            }
    }

    void savecosttimetocsv()
    {
        std::ofstream outFile;
        char date[120] = {0};
        // 以当前系统时间生成评估文件名称
        auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm *ptm = localtime(&tt);
        sprintf(date, "./%d-%02d-%02d %02d:%02d:%02d_costtime.csv",
                (int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
                (int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
        outFile.open(date, std::ios::out); // 打开模式可省略 
        for (size_t seq = 0;  seq < 2; seq++)
        {
            for (int i = 0; i < sceneslen[seq]; i++)
            {
                outFile << seq<< ", "<< i<< ", ";
                outFile << scenesprocesstime[seq][i] << ", ";
                outFile << std::endl;
            }
        }
        outFile.close();
      
    }

    void printSeqEvaluate(int pSeq)
    {
        // 地面评估指标
        //表示实际是地面点，且被分为地面点
        double TP = confusionMatrix[pSeq][0][0];
        //表示实际是非地面点，但是被分为地面点
        double FP = confusionMatrix[pSeq][1][0];
        //表示实际是地面点，但是被分为非地面点
        double FN = confusionMatrix[pSeq][0][1];
        //表示实际是非地面点，且被分为非地面点
        double TN = confusionMatrix[pSeq][1][1];

        // 精确度
        mPrecision[pSeq] = TP / (TP + FP);
        // 召回率
        mRecall[pSeq] = TP / (TP + FN);
        // F1
        mF1[pSeq] = 2 * TP / (2 * TP + FP + FN);
        // 准确度
        mAccuracy[pSeq] = (TP + TN) / (TP + FP + FN + TN);
        // 交并比
        mIou[pSeq] = TP/(TP+FP+FN);
        
        // 消耗时间
        mcosttime[pSeq] = segCostTime[pSeq] / (float)ptcount[pSeq];
        std::cout << std::endl;
        std::cout << "精确度：" << mPrecision[pSeq] * 100 << std::endl;
        std::cout << "召回率：" << mRecall[pSeq] * 100 << std::endl;
        std::cout << "F1：" << mF1[pSeq] * 100 << std::endl;
        std::cout << "准确度：" << mAccuracy[pSeq] * 100 << std::endl;
        std::cout << "交并比：" << mIou[pSeq] * 100 << std::endl;
        std::cout << "消耗时间：" << mcosttime[pSeq] << std::endl;
    }

    void saveEvaluateResultToCSV()
    {
        if (mPTCloud->rows == 64)
        {
            // 以当前系统时间生成评估文件名称
            auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            struct tm *ptm = localtime(&tt);
            char date[120] = {0};
            sprintf(date, "./%d-%02d-%02d %02d:%02d:%02d.csv",
                    (int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
                    (int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
            // 将评估结果写入文件
            std::ofstream outFile;
            outFile.open(date, std::ios::out); // 打开模式可省略
            outFile << std::endl;

            outFile << "seq., ";
            for (int pSeq = 0; pSeq <= 10; ++pSeq)
            {
                outFile << pSeq << ", ";
            }
            outFile << std::endl;
            // 输出精确率信息
            outFile << "Precision, ";
            for (int pSeq = 0; pSeq <= 10; ++pSeq)
            {
                outFile << mPrecision[pSeq] * 100 << ", ";
            }
            outFile << std::endl;
            // 输出召回率信息
            outFile << "Recall, ";
            for (int pSeq = 0; pSeq <= 10; ++pSeq)
            {
                outFile << mRecall[pSeq] * 100 << ", ";
            }
            outFile << std::endl;
            // 输出F1信息
            outFile << "F1, ";
            for (int pSeq = 0; pSeq <= 10; ++pSeq)
            {
                outFile << mF1[pSeq] * 100 << ", ";
            }
            outFile << std::endl;
            // 输出准确率信息
            outFile << "Accuracy, ";
            for (int pSeq = 0; pSeq <= 10; ++pSeq)
            {
                outFile << mAccuracy[pSeq] * 100 << ", ";
            }
            outFile << std::endl;
            // 输出交并比信息
            outFile << "IoU, ";
            for (int pSeq = 0; pSeq <= 10; ++pSeq)
            {
                outFile << mIou[pSeq] * 100 << ", ";
            }
            outFile << std::endl;
            
            // 输出消耗时间信息
            outFile << "costtime, ";
            for (int pSeq = 0; pSeq <= 10; ++pSeq)
            {
                outFile << mcosttime[pSeq] << ", ";
                // std::cout << total_segCostTime[pSeq] <<", ";
            }
            outFile << std::endl;
            outFile.close();
        }
    }
};