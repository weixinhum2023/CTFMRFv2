/************************
作者：黄伟鑫
日期：20220112
功能：环状栅格高度差算法
************************/
#pragma once

#include "./tool/PTCloudDataFrame.hpp"

#define TAN5 0.0875
#define TAN10 0.176
#define TAN20 0.36397
#define TAN30 0.5774

class ringGridHDiffNew
{
private:
    double gridYSeg[300]{};        // 环状栅格距离设置,用于环状栅格高度差
    int gridYSegMap[100001]{};     // 为栅格分割建立查找表
    float **distanceMap = nullptr; // 距离查找表
public:
    gridFactory mGridFac;        // 栅格对象
    PTCloud *mPTCloud = nullptr; // 点云对象
    explicit ringGridHDiffNew(PTCloud *getPTCloud)
    {
        mPTCloud = getPTCloud;
        mGridFac.initGrid(360, 300); // 初始化栅格，横向180，纵向60
        // 设置固定尺度栅格，沈师弟提出算法
        gridYSeg[0] = 0;
        gridYSeg[1] = 4.30;

        // 沈志豪师弟提出的方法
        for (int i = 2; i < 300; i++)
        {
            gridYSeg[i] = gridYSeg[i - 1] + 1;
        }

        // 为分段建立查找表
        for (int d1000 = 0; d1000 <= 100000; d1000++)
        {
            float d = (float)d1000 / 1000.0f;
            int gridY = getGridY(d);
            gridYSegMap[d1000] = gridY;
        }
    }

    // 栅格落点计算
    int getGridY(float ptDistance)
    {
        // 这里直接这么算会耗时4ms左右，如果改成查表法能大大加快速度
        for (int i = 0; i < 300; i++)
        {
            if (ptDistance >= gridYSeg[i] && ptDistance < gridYSeg[i + 1])
            {
                return i;
            }
        }

        return 299;
    }

    ~ringGridHDiffNew()
    {
        mGridFac.releaseGrid();
    }

    void run()
    {
        // 栅格初始化
        mGridFac.clearGrid();
        // 栅格高度统计
        for (int l = 0; l < mPTCloud->rows; l++)
        {
            for (int c = 0; c < mPTCloud->cols; c++)
            {
                if (mPTCloud->mPTCloudInfo[l][c].mergeMarkType == 0)
                {
                    continue;
                }
                /*************获得点对应的栅格坐标*************/
                // 获得栅格的x坐标
                unsigned int gridX = c / 5; // 按5个点代表1度，则25个点代表5度

                // 通过距离获得栅格的y坐标，由于计算距离消耗计算时间，因此改为查表法直接查表得到栅格y坐标
                /************正常计算************/
                // float d = sqrt(x * x + y * y);
                // unsigned int gridY = getGridY(mPTCloud->mPTCloudInfo[l][c].distance); //不直接计算而改用查表
                /*************查表法*************/
                unsigned int selDict = (unsigned int)(mPTCloud->mPTCloudInfo[l][c].distance * 1000);
                unsigned int gridY = gridYSegMap[selDict];

                mPTCloud->mPTCloudInfo[l][c].gridx = gridX;
                mPTCloud->mPTCloudInfo[l][c].gridy = gridY;

                // 记录栅格内中的点的行和列的值
                //                mGridFac.mGrid[gridY][gridX].ptLine[mGridFac.mGrid[gridY][gridX].density]=l;
                //                mGridFac.mGrid[gridY][gridX].ptCols[mGridFac.mGrid[gridY][gridX].density]=c;
                // mGridFac.mGrid[gridY][gridX].density++; // 计数
                if (mGridFac.mGrid[gridY][gridX].low > mPTCloud->mPTCloud[l][c].z)
                {
                    mGridFac.mGrid[gridY][gridX].low = mPTCloud->mPTCloud[l][c].z;
                    mGridFac.mGrid[gridY][gridX].lowest_l = l;
                    mGridFac.mGrid[gridY][gridX].lowest_c = c;
                }
            }
        }

        // 沈志豪师弟添加的栅格联系
        // 先设置最近的栅格最低点
        float groundheight = -1.6;

        for (std::size_t j = 0; j < 360; j++)
        {
            mGridFac.mGrid[0][j].low = mGridFac.mGrid[0][j].low > groundheight ? groundheight : mGridFac.mGrid[0][j].low;
            mGridFac.mGrid[0][j].getLowestValue = true;
        }

        // 地面点检测
        for (int i = 1; i < 119; i++)
        {
            for (int j = 1; j < 359; j++)
            {
                for (int h = -1; h <= 1; h++)
                {
                    int sel_l=i+h;
                    if(sel_l<0)
                        continue;
                    for (int k = -1; k <= 1; k++)
                    {
                        int sel_c=j+k;
                        if (sel_c < 0)
                                sel_c = 360 + sel_c;
                            else if (sel_c >= 360)
                                sel_c = sel_c - 360;
                    
                        if (k != 0)
                        {
                            if (mGridFac.mGrid[i][j].low > mGridFac.mGrid[sel_l][sel_c].low + TAN10)
                            {
                                mGridFac.mGrid[i][j].low = mGridFac.mGrid[sel_l][sel_c].low + TAN10;
                                // std::cout << "low: " << mGridFac.mGrid[i][j].low << std::endl;
                            }
                        }
                        // // 上一行
                        // if (mGridFac.mGrid[i][j].low > mGridFac.mGrid[i - 1][sel_c].low + TAN10)
                        // {
                        //     mGridFac.mGrid[i][j].low = mGridFac.mGrid[i - 1][sel_c].low + TAN10;
                        //     // std::cout << "low: " << mGridFac.mGrid[i][j].low << std::endl;
                        // }
                    }
                }
            }
        }
        // // 检查是否有因突然高低变化产生的错误地面点
        // for (int i = 1; i < 119; i++)
        // {
        //     for (int j = 1; j < 359; j++)
        //     {
        //         if (mGridFac.mGrid[i][j].low < mGridFac.mGrid[i][j - 1].low - TAN10)
        //         {
        //             mGridFac.mGrid[i][j].low = mGridFac.mGrid[i][j - 1].low - TAN10;
        //         }
        //     }
        //      for (int j = 1; j < 359; j++)
        //     {
        //         if (mGridFac.mGrid[i][j].low < mGridFac.mGrid[i][j + 1].low - TAN10)
        //         {
        //             mGridFac.mGrid[i][j].low = mGridFac.mGrid[i][j + 1].low - TAN10;
        //         }
        //     }
        // }

        // 依据栅格高度提取地面点
        for (int l = 0; l < mPTCloud->rows; l++)
        {
            for (int c = 0; c < mPTCloud->cols; c++)
            {
                if (mPTCloud->mPTCloudInfo[l][c].mergeMarkType == 0)
                {
                    mPTCloud->mPTCloudInfo[l][c].setType = 0;
                    continue;
                }
                // 取得栅格高度差
                unsigned int gridX = mPTCloud->mPTCloudInfo[l][c].gridx;
                unsigned int gridY = mPTCloud->mPTCloudInfo[l][c].gridy;
                mPTCloud->mPTCloudInfo[l][c].heightDif = mPTCloud->mPTCloud[l][c].z - mGridFac.mGrid[gridY][gridX].low;
                mPTCloud->mPTCloudInfo[l][c].lowestPt = mGridFac.mGrid[gridY][gridX].low;
                if (mPTCloud->rows == 64)
                {
                    // int density = 5;
                    //float gridHeightDif = 0.2;
                    float gridHeightDif = TAN10;
                    // if (mPTCloud->mPTCloudInfo[l][c].distance >= 30)
                    // {
                    //     gridHeightDif = TAN20;
                    // }
                    // if (mPTCloud->mPTCloudInfo[l][c].distance >= 40)
                    // {
                    //     gridHeightDif = TAN30;
                    // }
                    if (c > 830 && c < 950) // 帧间的交界处可能会有偏差，因此把这块地方的阈值加大
                    {
                        gridHeightDif = 0.5;
                    }
                    mPTCloud->mPTCloudInfo[l][c].theoreticalHeightDif = gridHeightDif;
                    if (mPTCloud->mPTCloudInfo[l][c].heightDif > gridHeightDif)
                    {
                        mPTCloud->mPTCloudInfo[l][c].setType = 20;
                    }
                    else
                    {
                        mPTCloud->mPTCloudInfo[l][c].setType = 10;
                    }
                }
                else if (mPTCloud->rows == 128)
                {
                    float gridHeightDif = 0.2;
                    mPTCloud->mPTCloudInfo[l][c].theoreticalHeightDif = gridHeightDif;
                    if (mPTCloud->mPTCloudInfo[l][c].heightDif > gridHeightDif)
                    {
                        mPTCloud->mPTCloudInfo[l][c].setType = 20;
                    }
                    else
                    {
                        mPTCloud->mPTCloudInfo[l][c].setType = 10;
                    }
                }
                else if (mPTCloud->rows == 32)
                {
                    float gridHeightDif = 0.165;

                    if (mPTCloud->mPTCloudInfo[l][c].heightDif > gridHeightDif)
                    {
                        mPTCloud->mPTCloudInfo[l][c].setType = 20;
                    }
                    else
                    {
                        mPTCloud->mPTCloudInfo[l][c].setType = 10;
                    }
                }
            }
        }
    }
};
