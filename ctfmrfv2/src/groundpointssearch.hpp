/************************
作者：黄伟鑫
日期：20240818
功能：地面搜索算法
************************/
#pragma once

#include "./tool/PTCloudDataFrame.hpp"
#include <forward_list>
#include <array>

class groundPointsSearch
{
private:

public:
    PTCloud *mPTCloud = nullptr; 
    cv::Mat *HeightRangeMap;     // 点云对象
    cv::Mat *EdgeRangeMap;      // canny边缘提取图像对象
    cv::Mat *EdgeRangeMapFiler;// 过滤后的边缘提取图像
    cv::Mat *HighconfidenceNongroundRangeMap;     //非地面点图像
    cv::Mat *ExpandedHighconfidenceNongroundRangeMap;  //膨胀后的图像
    cv::Mat *HighconfidenceGroundRangeMap;

    int *valuearray;
    int *imgDataInt;

    std::array<std::forward_list<cv::Point>, M_PT_COL * M_PT_ROW / 2> arr; // 开辟数组链表类型内存，存放各连通域点坐标，只允许放入图像小于等于1080Parray<forward_list<cv::Point>, M_PT_COL * M_PT_ROW / 2> arr; // 开辟数组链表类型内存，存放各连通域点坐标，只允许放入图像小于等于1080P

    cv::Mat element;
    explicit groundPointsSearch(PTCloud *getPTCloud)
    {
        mPTCloud=getPTCloud;

        // 图像处理对象
        HeightRangeMap = new cv::Mat(getPTCloud->rows, getPTCloud->cols, CV_8UC1);//原始图像
        EdgeRangeMap = new cv::Mat(getPTCloud->rows, getPTCloud->cols, CV_8UC1);
        EdgeRangeMapFiler = new cv::Mat(getPTCloud->rows, getPTCloud->cols, CV_8UC1);
        HighconfidenceNongroundRangeMap = new cv::Mat(getPTCloud->rows, getPTCloud->cols, CV_8UC1);
        ExpandedHighconfidenceNongroundRangeMap = new cv::Mat(getPTCloud->rows, getPTCloud->cols, CV_8UC1);
        HighconfidenceGroundRangeMap = new cv::Mat(getPTCloud->rows, getPTCloud->cols, CV_8UC1);

        //element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        // 连通域检测对象
        valuearray = new int[M_PT_COL * M_PT_ROW / 2]; // 设置标记数组最大可能找值为数组长度
        imgDataInt = new int[M_PT_COL * M_PT_ROW];     // 创建跟图像大小一样的int类型的内存对象
    }
    ~groundPointsSearch()
    {
        delete HeightRangeMap;
        delete EdgeRangeMap;
        delete EdgeRangeMapFiler;
        delete HighconfidenceNongroundRangeMap;
        delete ExpandedHighconfidenceNongroundRangeMap;
        delete HighconfidenceGroundRangeMap;
    }

    void OnePass(cv::Mat &img, int ptnum = 300) // 传出参数为茶叶对象左上和右下两点
    {
        memset(imgDataInt, 0, img.step * img.rows * sizeof(int)); // 设置初始值为0
        uchar *imgDataChar = img.data;
        int contourmark = 1;                        // 连通域标志
        int rowPoint;                               // 图像行指针偏移
        int lastRowPoint;                           // 图像上一行行指针偏移
        int colPoint;                               // 图像列指针偏移
        int lastColPoint;                           // 图像上一列列指针偏移
        int lastRowColPoint;                        // 图像上一行列指针偏移
        for (int row = 1; row < img.rows; row++) // 遍历图像行
        {
            rowPoint = row * img.step; // 图像行指针偏移计算
            lastRowPoint = (row - 1) * img.step;
            for (int col = 0; col < img.cols; col++)
            {
                colPoint = rowPoint + col; // 像素点本身
                if (col - 1 >= 0)
                    lastColPoint = rowPoint + col - 1; // 像素点左邻接
                else
                    lastColPoint = rowPoint + img.cols - 1;
                lastRowColPoint = lastRowPoint + col; // 像素点上邻接

                if (imgDataChar[colPoint] == 255) // 判断是不是连通域内的像素点
                {
                    // 如果不属于任何一个连通域
                    if (imgDataInt[lastRowColPoint] == 0 && imgDataInt[lastColPoint] == 0) // 该点邻接未被标值
                    {
                        valuearray[contourmark] = contourmark; // 标记该点
                        imgDataInt[colPoint] = contourmark;
                        contourmark++;
                    }
                    else // 该点邻接已被标值，则将邻接点中最小值标记值赋给他
                    {
                        if (imgDataInt[lastRowColPoint] != 0) // 其上面的值只有可能为0或者已被标定为其他，if成立则表明其被标定为其他
                        {
                            if (imgDataInt[lastColPoint] != 0) // 其左面的值只有可能为0或者已被标定为其他，if成立则表明其被标定为其他
                            {
                                if (valuearray[imgDataInt[lastColPoint]] > valuearray[imgDataInt[lastRowColPoint]]) // 左边大则用上边
                                {
                                    imgDataInt[colPoint] = valuearray[imgDataInt[lastRowColPoint]];
                                    valuearray[imgDataInt[lastColPoint]] = valuearray[imgDataInt[colPoint]]; // 标记该点
                                }
                                else
                                {
                                    imgDataInt[colPoint] = valuearray[imgDataInt[lastColPoint]];
                                    valuearray[imgDataInt[lastRowColPoint]] = valuearray[imgDataInt[colPoint]]; // 标记该点
                                }
                            }
                            else
                            {
                                imgDataInt[colPoint] = imgDataInt[lastRowColPoint];
                            }
                        }
                        else // 上面没有被标定，则为非连通域，则本值等于左值
                        {
                            imgDataInt[colPoint] = imgDataInt[lastColPoint];
                        }
                    }
                    arr[imgDataInt[colPoint]].push_front(cv::Point(col, row)); // 将连通域对象坐标放入链表中
                }
            }
        }
        for (int i = 1; i < contourmark; i++) // 将同一个连通域的对象映射表调整好，并最终整理完成连通域链表
        {
            valuearray[i] = valuearray[valuearray[i]];
            int srt = i;
            int dst = valuearray[valuearray[i]];
            if (srt != dst)
            {
                arr[dst].splice_after(arr[dst].before_begin(), arr[srt], arr[srt].before_begin(), arr[srt].end()); // 将同一连通域对象链表进行整合
            }
        }
        // 为不同的连通域上不同的颜色
        // 将图像数据设置为0

        img.setTo(0);
        // srand(time(0));
        for (int i = 1; i < contourmark; i++) // 将同一个连通域的对象映射表调整好，并最终整理完成连通域链表
        {
            if (!arr[i].empty())
            {
                // 打印链表长度
                auto begin_iter = arr[i].begin();
                auto end_iter = arr[i].end();
                auto size = std::distance(begin_iter, end_iter);
                // std::cout<<"size is: "<<size<<std::endl;
                int setcolor = 255;
                if (size < ptnum)
                {
                    setcolor = 0;
                }
                // uchar grayvalue = rand() % 256;
                bool set255 = false;
                for (auto &value : arr[i])
                {
                    if (value.y < 10 && setcolor == 100) // 前十根线不做计算
                    {
                        set255 = true;
                        setcolor = 255;
                        break;
                    }
                    img.at<unsigned char>(value.y, value.x) = setcolor;
                }
                if (set255) // 包含前10根线的连通域皆标注为地面
                {
                    for (auto &value : arr[i])
                    {
                        img.at<unsigned char>(value.y, value.x) = setcolor;
                    }
                }
            }
        }
        // 链表清空
        for (int i = 1; i < contourmark; i++)
        {
            if (!arr[i].empty())
            {
                arr[i].clear();
            }
        }
    }
    //第一篇文章的地面点搜索算法
    void paperone_run()
    {
        for (int l = 0; l < mPTCloud->rows; l++)
            for (int c = 0; c < mPTCloud->cols; c++)
            {
                if (mPTCloud->mPTCloudInfo[l][c].mergeMarkType == 0) //标记悬空点为背景
                {
                    mPTCloud->mPTCloudInfo[l][c].backOrFore = MASK_BG_COLOR;
                    continue;
                }
                if (mPTCloud->mPTCloudInfo[l][c].setType == 20)
                {
                    mPTCloud->mPTCloudInfo[l][c].backOrFore = MASK_FG_COLOR; //标记该点为前景
                }
                else //搜索地面点周围是否存在障碍物
                {
                    //orderpt->mptclout[l][c].backOrFore = MASK_BG_COLOR;//标记该点为背景
                    int i = 1;
                    int count = 1;
                    float Probability = 0;
                    //右半边
                    for (int r = -2; r <= 2; r++)
                    {
                        for (int s = -0; s < 10; s++)
                        {
                            if (r + l >= mPTCloud->rows || r + l < 0 || c + s < 0 || c + s >= mPTCloud->cols)
                            {
                                continue;
                            }
                            if (mPTCloud->mPTCloudInfo[r + l][c + s].mergeMarkType == 0)
                            {
                                continue;
                            }
                            if (mPTCloud->mPTCloudInfo[r + l][c + s].setType == 10)
                            {
                                i++;
                            }
                            count++;
                        }
                    }
                    Probability = i / (float)count;
                    i = 1;
                    count = 1;
                    //左半边
                    for (int r = -2; r <= 2; r++)
                    {
                        for (int s = -10; s < 0; s++)
                        {
                            if (r + l >= mPTCloud->rows || r + l < 0 || c + s < 0 || c + s >= mPTCloud->cols)
                            {
                                continue;
                            }
                            if (mPTCloud->mPTCloudInfo[r + l][c + s].mergeMarkType == 0)
                            {
                                continue;
                            }
                            if (mPTCloud->mPTCloudInfo[r + l][c + s].setType == 10)
                            {
                                i++;
                            }
                            count++;
                        }
                    }
                    //取较大的值，这样大部分地面点会被直接归结为地面
                    Probability = Probability < (i / (float)count) ? (i / (float)count) : Probability;

                    if (Probability > 0.95)
                    {
                        mPTCloud->mPTCloudInfo[l][c].backOrFore = MASK_BG_COLOR; //标记该点为背景
                    }
                    else
                    {
                        mPTCloud->mPTCloudInfo[l][c].backOrFore = 0; //标记该点为未知点
                    }
                }
            }

    }

    void run(int onepassvalue = 80)
    {
        /*****先遍历点云，将障碍物标注为高置信度障碍物,其他点暂时标注为高置信度地面点*****/
        for (int l = 0; l < mPTCloud->rows; l++)
        {
            for (int c = 0; c < mPTCloud->cols; c++)
            {
                if (mPTCloud->mPTCloudInfo[l][c].setType == 20)
                {
                    mPTCloud->mPTCloudInfo[l][c].backOrFore = MASK_FG_COLOR;
                }
                else
                {
                    mPTCloud->mPTCloudInfo[l][c].backOrFore = MASK_BG_COLOR;
                }
            }
        }
        /*****使用边沿提取算法提取图像的边沿部分*****/
        for (int l = 0; l < mPTCloud->rows; l++)
        {
            for (int c = 0; c < mPTCloud->cols; c++)
            {
                if (mPTCloud->mPTCloudInfo[l][c].mergeMarkType == 0)
                    HeightRangeMap->at<uchar>(l, c) = 0;
                else
                {
                    int height = int((mPTCloud->mPTCloud[l][c].z + 3) * 40);
                    if (mPTCloud->rows == 128)
                        height = int((mPTCloud->mPTCloud[l][c].z + 1) * 50);
                    if (height > 255)
                        height = 255;
                    else if (height < 0)
                        height = 0;
                    HeightRangeMap->at<uchar>(l, c) = height;
                }
            }
        }
        //cv::Canny(image, edge, 32, 64);
        cv::Canny(*HeightRangeMap, *EdgeRangeMap, 8, 16);
        // cv::imshow("",edge);
        // cv::waitKey(10);
        
        // 过滤靠近无效点的边缘
        for (int l = 1; l < mPTCloud->rows - 1; l++)
        {
            for (int c = 1; c < mPTCloud->cols - 1; c++)
            {
                if (EdgeRangeMap->at<uchar>(l, c) == 255)
                {
                    int ignorept = 0;
                    for (int i = -1; i <= 1; i++)
                        for (int j = -1; j <= 1; j++)
                        {
                            if (HeightRangeMap->at<uchar>(l + i, c + j) == 0)
                                ignorept++;
                        }
                    if (ignorept <= 1)
                        EdgeRangeMapFiler->at<uchar>(l, c) = 255;
                    else
                        EdgeRangeMapFiler->at<uchar>(l, c) = 0;
                }
                else
                {
                    EdgeRangeMapFiler->at<uchar>(l, c) = 0;
                }
            }
        }
        for (int l = 1; l < EdgeRangeMap->rows - 1; l++)
        {
            for (int c = 1; c < EdgeRangeMap->cols - 1; c++)
            {
                if (EdgeRangeMapFiler->at<uchar>(l, c) == 255)
                    mPTCloud->mPTCloudInfo[l][c].backOrFore = 0;
            }
        }

        /*****障碍物对象膨胀*****/
        for (int l = 0; l < HighconfidenceNongroundRangeMap->rows; l++)
        {
            for (int c = 0; c < HighconfidenceNongroundRangeMap->cols; c++)
            {
                if (mPTCloud->mPTCloudInfo[l][c].backOrFore == MASK_FG_COLOR)
                {
                    HighconfidenceNongroundRangeMap->at<uchar>(l, c) = 255;
                }
                else
                {
                    HighconfidenceNongroundRangeMap->at<uchar>(l, c) = 0;
                }
            }
        }
        dilate(*HighconfidenceNongroundRangeMap, *ExpandedHighconfidenceNongroundRangeMap, element);
        for (int l = 0; l < HighconfidenceNongroundRangeMap->rows; l++)
        {
            for (int c = 0; c < HighconfidenceNongroundRangeMap->cols; c++)
            {
                if (mPTCloud->mPTCloudInfo[l][c].backOrFore == MASK_BG_COLOR)
                {
                    if (ExpandedHighconfidenceNongroundRangeMap->at<uchar>(l, c) == 255 && mPTCloud->mPTCloudInfo[l][c].mergeMarkType != 0)
                    {
                        mPTCloud->mPTCloudInfo[l][c].backOrFore = 0;
                        // sstd::cout << "计算膨胀点" << std::endl;
                    }
                }
            }
        }
        // mTimerPerpaere.stop();
        // mTimerPerpaere.print("图割准备耗时：");

        /*********联通域检测*********/
        if (mPTCloud->rows != 32)
        {
            for (int l = 0; l < HighconfidenceGroundRangeMap->rows; l++)
            {
                for (int c = 0; c < HighconfidenceGroundRangeMap->cols; c++)
                {
                    if (mPTCloud->mPTCloudInfo[l][c].mergeMarkType == 0)
                    {
                        HighconfidenceGroundRangeMap->at<uchar>(l, c) = 0;
                    }
                    else
                    {
                        if (mPTCloud->mPTCloudInfo[l][c].backOrFore == MASK_BG_COLOR) // 考虑边缘影响
                        {
                            HighconfidenceGroundRangeMap->at<uchar>(l, c) = 255;
                        }
                        else
                        {
                            HighconfidenceGroundRangeMap->at<uchar>(l, c) = 0;
                        }
                    }
                }
            }
            // cv::flip(connectedComponentsMap, flip, 0);
            // cv::imshow("连通域检测前", flip);
            // cv::imwrite("连通域检测前.png", flip);
            OnePass(*HighconfidenceGroundRangeMap, onepassvalue); // 一遍遍历改进算法

            for (int l = 10; l < HighconfidenceGroundRangeMap->rows; l++) // 前10行不作考虑
            {
                for (int c = 0; c < HighconfidenceGroundRangeMap->cols; c++)
                {
                    if (mPTCloud->mPTCloudInfo[l][c].backOrFore == MASK_BG_COLOR && mPTCloud->mPTCloudInfo[l][c].mergeMarkType != 0)
                    {
                        if (HighconfidenceGroundRangeMap->at<uchar>(l, c) != 255)
                        {
                            mPTCloud->mPTCloudInfo[l][c].backOrFore = 0;
                        }
                    }
                }
            }
        }
    }
    void writeImageRun()
    {
        //std::cout<<"开始写入图像"<<std::endl;
        cv::Mat writeImage;
        cv::flip(*HeightRangeMap, writeImage, 0);
        cv::imwrite("HeightRangeMap.png", writeImage);
        cv::flip(*EdgeRangeMapFiler, writeImage, 0);
        cv::imwrite("EdgeRangeMapFiler.png", writeImage);
        cv::flip(*HighconfidenceNongroundRangeMap, writeImage, 0);
        cv::imwrite("HighconfidenceNongroundRangeMap.png", writeImage);
        cv::flip(*ExpandedHighconfidenceNongroundRangeMap, writeImage, 0);
        cv::imwrite("ExpandedHighconfidenceNongroundRangeMap.png", writeImage);
        cv::flip(*HighconfidenceGroundRangeMap, writeImage, 0);
        cv::imwrite("HighconfidenceGroundRangeMap.png", writeImage);
        //std::cout<<"图像写入完成"<<std::endl;
    }
};
