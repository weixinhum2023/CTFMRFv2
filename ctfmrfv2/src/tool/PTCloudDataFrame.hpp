/************************
作者：黄伟鑫
日期：20220112
功能：点云数据结构定义
************************/

#pragma once

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "Settings.hpp"

class pointX
{
public:
    float x = 0;
    float y = 0;
    float z = 0;
    float r = 0;
    int markType = 0;
    int obj = 0;
};

class pointXInfo
{
public:
    unsigned char mergeMarkType; // 合并后的标记分类，0为无效点，10为地面点，20为非地面点，30为忽视点
    unsigned char setType;       // 算法计算得到的分类
    float distance;              // 距离
    unsigned int gridx;          // 在栅格中的x位置
    unsigned int gridy;          // 在栅格中的y位置
    unsigned int lowest_line;//栅格中的最低点的线
    unsigned int lowest_cols;//栅格中的最低点的位置
    unsigned int heightest_line;//栅格中的最低点的线
    unsigned int heightest_cols;//栅格中的最低点的位置

    float heightDif;            // 高度差
    float theoreticalHeightDif; // 理论上可接受的高度差
    float lowestPt;             // 最低点

    unsigned char backOrFore; // 前景背景标志位
};

class PTCloud
{
public:
    pointX *mPTCloudData = nullptr;         // 点云对象，指向点云数据区
    pointX **mPTCloud = nullptr;            // 点云对象，二维索引
    pointXInfo *mPTCloudInfoData = nullptr; // 点云信息对象，指向点云信息数据区
    pointXInfo **mPTCloudInfo = nullptr;    // 点云信息对象，指向点云信息数据区
    /*点云对象的长宽*/
    int rows;
    int cols;

    /*点云图对象，用以进行并行图割*/
    pointX **mPTCloudMapData; // 点云指针对象
    pointX ***mPTCloudMap;    // 点云指针对象地图

    pointXInfo **mPTCloudInfoMapData; // 点云信息指针对象
    pointXInfo ***mPTCloudInfoMap;    // 点云信息指针对象地图

public:
    PTCloud(int gRows, int gCols) // 构造函数
    {
        rows = gRows;
        cols = gCols;
        mPTCloud = new pointX *[rows];
        mPTCloudData = new pointX[rows * cols];
        mPTCloudInfo = new pointXInfo *[rows];
        mPTCloudInfoData = new pointXInfo[rows * cols];

        mPTCloudInfoMap = new pointXInfo **[rows];
        mPTCloudInfoMapData = new pointXInfo *[rows * cols];

        mPTCloudMap = new pointX **[rows];
        mPTCloudMapData = new pointX *[rows * cols];

        for (int i = 0; i < rows; i++) // 建立二维索引的映射关系
        {
            mPTCloud[i] = &mPTCloudData[i * cols];
            mPTCloudInfo[i] = &mPTCloudInfoData[i * cols];

            mPTCloudInfoMap[i] = &mPTCloudInfoMapData[i * cols];
            mPTCloudMap[i] = &mPTCloudMapData[i * cols];
        }
    }

    ~PTCloud() // 析构函数，回收资源
    {
        delete[] mPTCloudData;
        delete[] mPTCloud;
        delete[] mPTCloudInfoData;
        delete[] mPTCloudInfo;
        delete[] mPTCloudInfoMapData;
        delete[] mPTCloudInfoMap;
        delete[] mPTCloudMapData;
        delete[] mPTCloudMap;
    }

    // 数据预处理，对标记进行合并，然后将无效的点（比如地面以下的杂点和车辆头上的杂点）标记为0
    void dataPreprocessing() const
    {
        // int pt0count=0;
        // 将标注类型合并为地面、关键障碍物和普通障碍物三种类型
        for (int l = 0; l < rows; l++)
        {
            for (int c = 0; c < cols; c++)
            {
                if (rows == 64 || rows == 128)
                {
                    if (mPTCloud[l][c].markType != 0 && mPTCloud[l][c].markType != 1 && mPTCloud[l][c].z > -5)
                    {
                        mPTCloudInfo[l][c].distance = sqrtf(mPTCloud[l][c].x * mPTCloud[l][c].x + mPTCloud[l][c].y * mPTCloud[l][c].y);
                        //处理地面以下的点
                        if (rows == 64)
                        {
                            /*******SegmanticKITTI需要额外忽略的点*******/
                            if (l <= 5 && mPTCloud[l][c].z < -1.8)
                            {
                                mPTCloudInfo[l][c].mergeMarkType = 0; // 无效点
                                // pt0count++;
                                continue;
                            }
                            if (l < 20 && mPTCloud[l][c].z < -3)
                            {
                                mPTCloudInfo[l][c].mergeMarkType = 0; // 无效点
                                // pt0count++;
                                continue;
                            }
                            if (mPTCloudInfo[l][c].distance >= 50 || mPTCloudInfo[l][c].distance <= 0)
                            {
                                mPTCloudInfo[l][c].mergeMarkType = 0; // 无效点
                                // pt0count++;
                                continue;
                            }
                            if (mPTCloudInfo[l][c].distance < 1 && mPTCloud[l][c].y > -3.5 && mPTCloud[l][c].y < 1.7)
                            {
                                mPTCloudInfo[l][c].mergeMarkType = 0; // 无效点
                                // pt0count++;
                                continue;
                            }
                        }
                        
                        //设置分类点
                        if (mPTCloud[l][c].markType == 40 || mPTCloud[l][c].markType == 44|| mPTCloud[l][c].markType == 48|| mPTCloud[l][c].markType == 49|| mPTCloud[l][c].markType == 60|| mPTCloud[l][c].markType == 72)
                        {
                            mPTCloudInfo[l][c].mergeMarkType = 10; // 地面
                        }
                        else if(mPTCloud[l][c].markType == 70)//植被定义为忽略，这样做不合理，在很多图中植被占了大多数
                        {
                            mPTCloudInfo[l][c].mergeMarkType = 20; // 障碍物
                        }
                        else
                        {
                            mPTCloudInfo[l][c].mergeMarkType = 20; // 障碍物
                        }
                    }
                    else
                    {
                        mPTCloudInfo[l][c].mergeMarkType = 0; // 无效点
                    }
                }
            }
        }
    }
};

// 栅格操作类
class gridFactory
{
    // 单个栅格对象
    struct CELL
    {
        float low; // 最低高度
        float high;  //最高高度
        int density; // 存在点数

        int lowest_l;               // 最低点的行数
        int lowest_c;               // 最低点的列数

        int heightest_l;            // 最高点的行数
        int heightest_c;               // 最高点的列数
        
        bool getLowestValue; // 获得最低点的有效值标记位
        bool isground;//只存在地面点的栅格
        float groundHeight;  // 猜测可能的地面高度
    };

public:
    CELL **mGrid{};        // 栅格二维指针对象
    CELL *mGridData{};     // 栅格指针对象
    CELL *mGridDataCopy{}; // 栅格指针对象
    int gw = 0;            // 栅格宽度
    int gh = 0;            // 栅格高度
    // 栅格初始化函数,输入栅格的长宽
    void initGrid(int w, int h)
    {
        mGridData = new CELL[w * h];
        mGridDataCopy = new CELL[w * h];
        mGrid = new CELL *[h];
        for (int i = 0; i < h; i++)
        {
            mGrid[i] = &mGridData[i * w];
        }
        gw = w;
        gh = h;
        // 设置初始值
        for (int i = 0; i < gh; i++)
        {
            for (int j = 0; j < gw; j++)
            {
                mGrid[i][j].high = -1000;
                mGrid[i][j].low = 10000;
                mGrid[i][j].density = 0;
                mGrid[i][j].getLowestValue = false;
                mGrid[i][j].isground = false;

                mGrid[i][j].lowest_l = 1000;
                mGrid[i][j].lowest_c = 1000;
                mGrid[i][j].heightest_l = -1000;
                mGrid[i][j].heightest_c = -1000;
                mGrid[i][j].groundHeight = 100;
            }
        }
        // 设置初始值存储对象
        memcpy(mGridDataCopy, mGridData, sizeof(CELL) * gw * gh);
    }

    // 栅格重置
    void clearGrid() const
    {
        //memcpy(mGridData, mGridDataCopy, sizeof(CELL) * gw * gh);
        for (int i = 0; i < gh; i++)
        {
            for (int j = 0; j < gw; j++)
            {
                //mGrid[i][j].high = -1000;
                mGrid[i][j].low = 10000;
                mGrid[i][j].high = -10000;
                mGrid[i][j].density = 0;
                mGrid[i][j].getLowestValue=false;
                mGrid[i][j].isground = false;

                mGrid[i][j].lowest_l=1000;
                mGrid[i][j].lowest_c=1000;
                mGrid[i][j].heightest_l=-1000;
                mGrid[i][j].heightest_c=-1000;
                mGrid[i][j].groundHeight=100;
            }
        }
    }

    // 栅格对象释放函数
    void releaseGrid() const
    {
        delete[] mGridDataCopy;
        delete[] mGridData;
        delete[] mGrid;
    }
};

// 标注对象与颜色的映射关系
class labelMap
{
public:
    std::string labels[260];           // 标签
    unsigned char color_map[260][3]{}; // 标签对应颜色
    labelMap()
    {
        //char markPath[100];
        //sprintf(markPath, "./src/toolset/labelMap.txt");
        std::ifstream infile;
        //infile.open(markPath, std::ios::in);
        infile.open(labelMapPath.c_str(), std::ios::in);
        std::string getLabels;
        std::getline(infile, getLabels, '\n'); // label
        for (size_t i = 1; i < 35; i++)
        {
            getline(infile, getLabels, '\n');
            std::string sNum = getLabels.substr(0, getLabels.find_first_of(':'));
            int num = std::stoi(sNum);
            // cout << num << endl;
            getLabels = getLabels.substr(getLabels.find_first_of(':') + 3, getLabels.find_last_of('\"') - getLabels.find_first_of(':') - 3);
            labels[num] = getLabels;
            // cout << labels[num] << endl;
        }
        std::string getColorMap;
        std::getline(infile, getColorMap, '\n'); // color_map # bgr
        for (size_t i = 1; i < 35; i++)
        {
            getline(infile, getColorMap, '\n');
            std::string sNum = getColorMap.substr(0, getColorMap.find_first_of(':'));
            int num = std::stoi(sNum);
            // cout << num << endl;
            getColorMap = getColorMap.substr(getColorMap.find_first_of(':') + 3, getColorMap.length() - getColorMap.find_first_of(':') - 3 - 1);
            std::string sR = getColorMap.substr(0, getColorMap.find_first_of(','));
            color_map[num][0] = std::stoi(sR);
            getColorMap = getColorMap.substr(getColorMap.find_first_of(',') + 1);
            std::string sG = getColorMap.substr(0, getColorMap.find_first_of(','));
            color_map[num][1] = std::stoi(sG);
            std::string sB = getColorMap.substr(getColorMap.find_first_of(',') + 1);
            color_map[num][2] = std::stoi(sB);
            // cout << (int)color_map[num][2] << endl;
        }
        infile.close();
    }
};

// 合并标签和算法检测结构标签对应颜色
class typeMap
{
public:
    unsigned char type_map[260][3]{}; // 检测标签对应颜色
    // 地面分割相关对象显示
    typeMap()
    {

        // 地面点着色
        type_map[10][0] = 14;
        type_map[10][1] = 204;
        type_map[10][2] = 114;
        // 关键障碍物点着色（由于检测结果无法区分关键障碍物点和普通障碍物点，因此检测结果统一为20）
        type_map[20][0] = 250;
        type_map[20][1] = 67;
        type_map[20][2] = 67;
        // 植物着色
        type_map[30][0] = 0;
        type_map[30][1] = 250;
        type_map[30][2] = 0;

        // 背景着色
        type_map[128][0] = 14;
        type_map[128][1] = 204;
        type_map[128][2] = 114;
        // 前景着色
        type_map[250][0] = 250;
        type_map[250][1] = 67;
        type_map[250][2] = 67;
        // 未知分类点
        type_map[0][0] = 67;
        type_map[0][1] = 67;
        type_map[0][2] = 250;
    }
};