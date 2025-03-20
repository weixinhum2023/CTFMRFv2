/************************
作者：黄伟鑫
日期：20220112
功能：从磁盘中读取点云文件
************************/

#pragma once
#include "PTCloudDataFrame.hpp"

#include <fstream>
#include <cstdio>
#include <chrono> //时间相关头文件
#include <thread>
#include <cstring>

#define DATA_BUF 30
class PTCloudRead
{
public:
    PTCloud *mPTCloud;                  // 点云对象
    unsigned int mSeq{};                // 系列
    unsigned int mPTCloudNum{};         // 点云索引
    char ptCloudPath[200]{};            // 点云地址
    bool haveReadData = false;          // 点云读取标志位
    unsigned int dataQuantity[850]{};   // 系列点云的数量
    unsigned int preReadPTCloudNum = 0; // 记录当前预读取点云读取的位置
    unsigned int preReadSeqNum = 0;     // 记录当前预读取系列读取的位置
    unsigned int readPTCloudNum = 0;    // 记录读取点云读取的位置
    unsigned int readSeqNum = 0;        // 记录读取系列读取的位置

    pointX *readPTCloudBuf[DATA_BUF]{}; // 数据对象指针，用以异步读取数据
    bool dataWaiteToBeRead[DATA_BUF]{};
    int writePt = 0; // 写数据指针
    int readPt = 0;  // 读数据指针
    bool runPreReadPTCloud = true;
    bool PreReadPTCloudHaveStop = false;
    int datasetSelect = 0; // SemanticKITTI为0
    void preReadPTCloud()  // 预先读取点云到内存中
    {
        while (preReadPTCloudNum <= dataQuantity[preReadSeqNum])
        {
            if (!dataWaiteToBeRead[writePt])
            {
                if (datasetSelect == 0)
                    //sprintf(ptCloudPath, "/home/weixinhum/第二篇文章使用点云数据集/data_odometry_semantickitti/%02d/semantickitti/%06d.binX", preReadSeqNum, preReadPTCloudNum); // 设置点云地址
                    sprintf(ptCloudPath, "%s/data_odometry_semantickitti/%02d/semantickitti/%06d.binX",rootPath.c_str(), preReadSeqNum, preReadPTCloudNum); // 设置点云地址
               
                // 读取点云
                readPTCloudToBuf(ptCloudPath, readPTCloudBuf[writePt]);
                // 设置标志位
                preReadPTCloudNum++;
                // 设置数据标志
                dataWaiteToBeRead[writePt] = true;
                writePt++;
                if (writePt == DATA_BUF)
                {
                    writePt = 0;
                }
            }
            else
            {
                if (!runPreReadPTCloud)
                {
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 线程休眠一段时间
            }
        }
        PreReadPTCloudHaveStop = true;
    }
    
    void initQuantity()
    {

        if (mPTCloud->rows == 64) // SemanticKITTI
        {
            dataQuantity[0] = 4540;
            dataQuantity[1] = 1100;
            dataQuantity[2] = 4660;
            dataQuantity[3] = 800;
            dataQuantity[4] = 270;
            dataQuantity[5] = 2760;
            dataQuantity[6] = 1100;
            dataQuantity[7] = 1100;
            dataQuantity[8] = 4070;
            dataQuantity[9] = 1590;
            dataQuantity[10] = 1200;
            datasetSelect = 0;
        }

        // 初始化内存
        writePt = 0;
        readPt = 0;
        for (size_t i = 0; i < DATA_BUF; i++)
        {
            readPTCloudBuf[i] = new pointX[mPTCloud->cols * mPTCloud->rows];
            dataWaiteToBeRead[i] = false;
        }
    }

    PTCloudRead(PTCloud *getPTCloud, unsigned int seq) // 获取一个系列点云对象，需要输入系列
    {
        mPTCloud = getPTCloud;
        mSeq = seq;
        preReadPTCloudNum = 0;
        preReadSeqNum = seq;
        initQuantity();
        // 开启异步IO
        std::thread preReadThread(&PTCloudRead::preReadPTCloud, this); // 开启接收线程
        preReadThread.detach();                                        // 程序不会阻塞
    }

    ~PTCloudRead()
    {
        runPreReadPTCloud = false;
        while (!PreReadPTCloudHaveStop)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 线程休眠一段时间
        }

        for (auto &i : readPTCloudBuf)
        {
            delete[] i;
        }
        //std::cout << "读取程序退出" << std::endl;
        //std::cout << std::endl;
    }

    void readPTCloudToBuf(char *Path, pointX *dataBuf) const
    {
        /***用ifstream+设置缓存空间的形式读取文件***/
        std::ifstream readPTFile(Path, std::ios::in | std::ios::binary);
        readPTFile.rdbuf()->pubsetbuf((char *)dataBuf, sizeof(pointX) * mPTCloud->cols * mPTCloud->rows);
        readPTFile.read((char *)dataBuf, sizeof(pointX) * mPTCloud->cols * mPTCloud->rows);
        readPTFile.close();
    }

    bool readPTCloud(unsigned int &seq, unsigned int &seqPTCloudNum)
    {
        if (readPTCloudNum <= dataQuantity[mSeq])
        {
            // 从异步数据区中拷贝点云数据
            // 等待异步线程写数据
            while (!dataWaiteToBeRead[readPt])
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            // 拷贝数据
            memcpy(mPTCloud->mPTCloudData, readPTCloudBuf[readPt], sizeof(pointX) * mPTCloud->cols * mPTCloud->rows);
            // 读取信息显示
            std::cout << "\r\033[k"
                      << "seq: " << mSeq << "  " << readPTCloudNum << "/" << dataQuantity[mSeq];
            std::fflush(stdout);

            // 更新序列和帧对象
            seq = mSeq;
            seqPTCloudNum = readPTCloudNum;
            // 设置标志位
            readPTCloudNum++;
            // 设置读取指针
            dataWaiteToBeRead[readPt] = false;
            readPt++;
            if (readPt == DATA_BUF)
            {
                readPt = 0;
            }
            return true;
        }
        else
        {
            return false;
        }
        return false;
    }
};