/*
日期：2020年7月27日
作者：weixinhum
须知：单位是m
说明：图割算法，用于地面分割和聚类
*/
#pragma once
#include "gcgraphMy.h"

#define MASK_BG_COLOR 128
#define MASK_FG_COLOR 250

// #define MIN(a, b) ((a < b) ? a : b)
// #define MAX(a, b) ((a > b) ? a : b)
#define SWAP(a, b, t) {t = a; a = b; b = t;}

class GCApplication
{
public:
    int rows;
    int cols;

    float FgPHist[256];
    float BgPHist[256];
    float gamma;
    float lambda;
    float beta;

    int vtxCount;  //节点数目
    int edgeCount; //边数目

    //聚类相关的对象
    //记录点间的边的编号对象
    int *leftEdge;         //点与左边点的边的编号
    int *upleftEdge;       //点与左上边点的编号
    int *upEdge;           //点与上边点的编号
    int *uprightEdge;      //点与右上边点的编号
    int *clusterlist;      //编号索引表
    int *finalclusterlist; //最终列别标记

    cv::Mat leftW, upleftW, upW, uprightW;

    float **dictionary_edgevalue;//二维数组，用于查表获得边的损失

    // inline double Fast_Distance_2D(double x, double y)
    // {
    //     // this function computes the distance from 0,0 to x,y with 3.5% error
    //     // fist compute the absolute value of x,y
    //     x = abs(x);
    //     y = abs(y);
    //     // compute the minimum of x,y
    //     double mn = MIN(x, y);
    //     // return the distance
    //     return (x + y - (mn >> 1) - (mn >> 2) + (mn >> 4));
    // } // end Fast_Distance_2D

    inline int getdistance(pointX *pt1, pointX *pt2)
    {
        return int(sqrt((pt1->x - pt2->x) * (pt1->x - pt2->x) + (pt1->y - pt2->y) * (pt1->y - pt2->y))*100+0.5);
    }

    float get3Ddistance(pointX &pt1, pointX &pt2)
    {
        float dis = sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
        return dis;
    }

    bool valid(pointXInfo *pt) //有效点判断
    {
        if (pt->mergeMarkType == 0)
        {
            return false;
        }
        return true;
    }

    //计算马尔科夫随机场的点间连接权重
    void calcNWeights(PTCloud *orderpt, cv::Mat &leftW, cv::Mat &upleftW, cv::Mat &upW, cv::Mat &uprightW, float beta)
    {
        float value = 0;
        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                //如果距离太大直接将连接代价设置为无穷大
                if (x - 1 >= 0 && valid(orderpt->mPTCloudInfoMap[y][x]) && valid(orderpt->mPTCloudInfoMap[y][x - 1])) // left
                {
                    int diff = int(abs(orderpt->mPTCloudMap[y][x]->z - orderpt->mPTCloudMap[y][x - 1]->z)*100+0.5);
                    //float dist = getdistance(orderpt->mPTCloudMap[y][x], orderpt->mPTCloudMap[y][x - 1]);
                    int dist=getdistance(orderpt->mPTCloudMap[y][x], orderpt->mPTCloudMap[y][x - 1]);
                    //leftW.at<float>(y, x) = exp(-beta * diff * diff * dist) * dist;
                    //std::cout<<"dist:"<<int(dist*100)<<"   diff:"<<int(diff*100)<<std::endl;
                    leftW.at<float>(y, x) = dictionary_edgevalue[dist][diff];
                }
                else
                    leftW.at<float>(y, x) = value;
                if (x - 1 >= 0 && y - 1 >= 0 && valid(orderpt->mPTCloudInfoMap[y][x]) && valid(orderpt->mPTCloudInfoMap[y - 1][x - 1])) // upleft
                {
                    int diff = int(abs(orderpt->mPTCloudMap[y][x]->z - orderpt->mPTCloudMap[y - 1][x - 1]->z)*100+0.5);
                    int dist = getdistance(orderpt->mPTCloudMap[y][x], orderpt->mPTCloudMap[y - 1][x - 1]);
                    //upleftW.at<float>(y, x) = exp(-beta * diff * diff * dist) * dist;
                    upleftW.at<float>(y, x) = dictionary_edgevalue[dist][diff];
                }
                else
                    upleftW.at<float>(y, x) = value;
                if (y - 1 >= 0 && valid(orderpt->mPTCloudInfoMap[y][x]) && valid(orderpt->mPTCloudInfoMap[y - 1][x])) // up
                {
                    int diff = int(abs(orderpt->mPTCloudMap[y][x]->z - orderpt->mPTCloudMap[y - 1][x]->z)*100+0.5);
                    int dist = getdistance(orderpt->mPTCloudMap[y][x], orderpt->mPTCloudMap[y - 1][x]);
                    //upW.at<float>(y, x) = exp(-beta * diff * diff * dist) * dist;
                    upW.at<float>(y, x) = dictionary_edgevalue[dist][diff];
                }
                else
                    upW.at<float>(y, x) = value;
                if (x + 1 < cols && y - 1 >= 0 && valid(orderpt->mPTCloudInfoMap[y][x]) && valid(orderpt->mPTCloudInfoMap[y - 1][x + 1])) // upright
                {
                    int diff = int(abs(orderpt->mPTCloudMap[y][x]->z - orderpt->mPTCloudMap[y - 1][x + 1]->z)*100+0.5);
                    int dist = getdistance(orderpt->mPTCloudMap[y][x], orderpt->mPTCloudMap[y - 1][x + 1]);
                    //uprightW.at<float>(y, x) = exp(-beta * diff * diff * dist) * dist;
                    uprightW.at<float>(y, x) = dictionary_edgevalue[dist][diff];
                }
                else
                    uprightW.at<float>(y, x) = value;
            }
        }
    }

    GCApplication(unsigned int grows, unsigned int gcols)
    {
        rows = grows;
        cols = gcols;

        leftW.create(rows, cols, CV_32FC1);
        upleftW.create(rows, cols, CV_32FC1);
        upW.create(rows, cols, CV_32FC1);
        uprightW.create(rows, cols, CV_32FC1);
        //参数初始化
        lambda = 10000;
        beta = 1;

        //初始化聚类对象
        // leftEdge = new int[rows * cols];
        // upleftEdge = new int[rows * cols];
        // upEdge = new int[rows * cols];
        // uprightEdge = new int[rows * cols];
        clusterlist = new int[rows * cols];
        finalclusterlist = new int[rows * cols];

        //创建一个二维数组，分别以距离和差值为索引，用于查表获得边损失
        int distlen = 10000;
        int difflen = 2100;
          
        dictionary_edgevalue = new float *[distlen];
        for (int dist = 0; dist < distlen; dist++)
        {
            dictionary_edgevalue[dist] = new float [difflen];
            for (int diff = 0; diff < difflen; diff++)//设置最大相差10m
            {
                float m_diff=diff/100.0f;
                float m_dist=dist/100.0f;
                if(m_dist==0)
                    m_dist=100;
                else
                    m_dist=1/m_dist;
                dictionary_edgevalue[dist][diff] = exp(-beta * m_diff * m_diff * m_dist) * m_dist;//当前文章使用，除以距离的版本
                //dictionary_edgevalue[dist][diff] = exp(-beta * m_diff * m_diff * m_dist);//不除以距离的版本
            }
        }
       
    }

    ~GCApplication() //释放对象
    {
        // delete[] leftEdge;
        // delete[] upleftEdge;
        // delete[] upEdge;
        // delete[] uprightEdge;
        delete[] clusterlist;
        delete[] finalclusterlist;

        //删除创建的二维数组
        for (int i = 0; i < 5000; i++)
        {
            delete []dictionary_edgevalue[i];            
        }
        delete [] dictionary_edgevalue;
    }

    void calSeedPHist(PTCloud *orderpt) //直方图统计，如果没有值则设为0.00001
    {
        int nFgNum = 0; //
        int nBgNum = 0; //
        memset(FgPHist, 0, 256 * sizeof(float));
        memset(BgPHist, 0, 256 * sizeof(float));

        cv::Point p;
        for (int l = 0; l < rows; l++)
            for (int c = 0; c < cols; c++)
            {
                if (orderpt->mPTCloudInfoMap[l][c]->mergeMarkType == 0) //标记悬空点为背景
                {
                    orderpt->mPTCloudInfoMap[l][c]->backOrFore = MASK_BG_COLOR;
                    continue;
                }
                if (orderpt->mPTCloudInfoMap[l][c]->setType == 20)
                {
                    orderpt->mPTCloudInfoMap[l][c]->backOrFore = MASK_FG_COLOR; //标记该点为前景

                    int pix = int((orderpt->mPTCloudMap[l][c]->z + 3.5) * 40);
                    pix = pix < 255 ? pix : 255;
                    pix = pix >= 0 ? pix : 0;
                    FgPHist[pix]++;
                    nFgNum++;
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
                            if (r + l >= rows || r + l < 0 || c + s < 0 || c + s >= cols)
                            {
                                continue;
                            }
                            if (orderpt->mPTCloudInfoMap[r + l][c + s]->mergeMarkType == 0)
                            {
                                continue;
                            }
                            if (orderpt->mPTCloudInfoMap[r + l][c + s]->setType == 10)
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
                            if (r + l >= rows || r + l < 0 || c + s < 0 || c + s >= cols)
                            {
                                continue;
                            }
                            if (orderpt->mPTCloudInfoMap[r + l][c + s]->mergeMarkType == 0)
                            {
                                continue;
                            }
                            if (orderpt->mPTCloudInfoMap[r + l][c + s]->setType == 10)
                            {
                                i++;
                            }
                            count++;
                        }
                    }
                    //取较大的值，这样大部分地面点会被直接归结为地面
                    Probability = Probability < (i / (float)count) ? (i / (float)count) : Probability;

                    if (Probability > 0.95)//20241216从0.95->0.99
                    {
                        orderpt->mPTCloudInfoMap[l][c]->backOrFore = MASK_BG_COLOR; //标记该点为背景

                        int pix = int((orderpt->mPTCloudMap[l][c]->z + 3.5) * 40);
                        pix = pix < 255 ? pix : 255;
                        pix = pix >= 0 ? pix : 0;
                        BgPHist[pix]++;
                        nBgNum++;
                    }
                    else
                    {
                        orderpt->mPTCloudInfoMap[l][c]->backOrFore = 0; //标记该点为未知点
                    }
                }
            }
        nFgNum = nFgNum > 0 ? nFgNum : 1;
        nBgNum = nBgNum > 0 ? nBgNum : 1;
        //归一化并防止除0
        for (int j = 0; j < 256; j++)
        {
            FgPHist[j] = FgPHist[j] / nFgNum;
            FgPHist[j] = FgPHist[j] < 0.00001 ? 0.00001 : FgPHist[j];
            //认为不是前景就是背景
            BgPHist[j] = BgPHist[j] / nBgNum;
            BgPHist[j] = BgPHist[j] < 0.00001 ? 0.00001 : BgPHist[j];
        }
    }

    void calSeedPHistPaper2(PTCloud *orderpt) //直方图统计，如果没有值则设为0.00001
    {
        int nFgNum = 0; //
        int nBgNum = 0; //
        memset(FgPHist, 0, 256 * sizeof(float));
        memset(BgPHist, 0, 256 * sizeof(float));

        cv::Point p;
        for (int l = 0; l < rows; l++)
            for (int c = 0; c < cols; c++)
            {
                if (orderpt->mPTCloudInfoMap[l][c]->mergeMarkType == 0) //标记悬空点为背景
                {
                    continue;
                }
                if (orderpt->mPTCloudInfoMap[l][c]->backOrFore == MASK_FG_COLOR)
                {
                    int pix = int((orderpt->mPTCloudMap[l][c]->z + 3.5) * 40);
                    pix = pix < 255 ? pix : 255;
                    pix = pix >= 0 ? pix : 0;
                    FgPHist[pix]++;
                    nFgNum++;
                }
                else if (orderpt->mPTCloudInfoMap[l][c]->backOrFore == MASK_BG_COLOR)
                {
                    int pix = int((orderpt->mPTCloudMap[l][c]->z + 3.5) * 40);
                    pix = pix < 255 ? pix : 255;
                    pix = pix >= 0 ? pix : 0;
                    BgPHist[pix]++;
                    nBgNum++;
                }
            }
        nFgNum = nFgNum > 0 ? nFgNum : 1;
        nBgNum = nBgNum > 0 ? nBgNum : 1;
        //归一化并防止除0
        for (int j = 0; j < 256; j++)
        {
            FgPHist[j] = FgPHist[j] / nFgNum;
            FgPHist[j] = FgPHist[j] < 0.00001 ? 0.00001 : FgPHist[j];
            //认为不是前景就是背景
            BgPHist[j] = BgPHist[j] / nBgNum;
            BgPHist[j] = BgPHist[j] < 0.00001 ? 0.00001 : BgPHist[j];
        }
    }

    void graphConstruct(PTCloud *orderpt, GCGraphMy<float> &graph, int useProperties) //创建图像结构和连接关系
    {
        // timer mTimercalcNWeights;
        // mTimercalcNWeights.start();
        calcNWeights(orderpt, leftW, upleftW, upW, uprightW, beta);
        // mTimercalcNWeights.stop();
        // mTimercalcNWeights.print("构建边连接耗时：");

        if (useProperties == 1) //第一篇文章，使用临近点计算点被分配为地面点或者未知分类点+随后用概率直方图+区域项链接
        {
            //std::cout<<"1"<<std::endl;
            calSeedPHist(orderpt); //直方图概率统计+高置信度点分类->第一篇文章
        }
        if (useProperties == 2) //直接用第二篇文章的高置信度点搜索结果+概率直方图+区域项链接
        {
            //std::cout<<"1"<<std::endl;
            calSeedPHistPaper2(orderpt); //直方图概率统计+高置信度点分类->第一篇文章
        }

        cv::Point p;
        //float a = 0;//比重
        //float a = 0.00000000000000000000000001; //比重
        float a = 1; //比重
        int vtxIdx = 0;
        int edgenum = 0;

        // memset(leftEdge, 0, rows * cols * sizeof(int));
        // memset(upleftEdge, 0, rows * cols * sizeof(int));
        // memset(upEdge, 0, rows * cols * sizeof(int));
        // memset(uprightEdge, 0, rows * cols * sizeof(int));
        float fromSource, toSink;
        for (p.y = 0; p.y < rows; p.y++)
        {
            for (p.x = 0; p.x < cols; p.x++)
            {
                if (orderpt->mPTCloudInfoMap[p.y][p.x]->backOrFore == MASK_FG_COLOR) //即障碍物
                {
                    fromSource = lambda;
                    toSink = 0;
                }
                else if (orderpt->mPTCloudInfoMap[p.y][p.x]->backOrFore == MASK_BG_COLOR)
                {
                    fromSource = 0;
                    toSink = lambda;
                }
                else
                {
                    if (useProperties==1 || useProperties==2) //使用区域项链接
                    {
                        //用直方图概率计算一个点属于前景或者背景的概率
                        int pix = int((orderpt->mPTCloudMap[p.y][p.x]->z + 3.5) * 40);
                        uchar color = pix < 255 ? pix : 255;
                        fromSource = -a * log(calBgdPrioriCost(color));
                        toSink = -a * log(calFgdPrioriCost(color));
                        // fromSource = -a * log(0.6);
                        // toSink = -a * log(0.4);
                    }
                    else //断开区域项链接
                    {
                        fromSource = 0;
                        toSink = 0;
                    }
                }
                graph.addTermWeights(vtxIdx, fromSource, toSink);
                // set n-weights,每个点只需要与左上4个点进行边连接即可,这样可以不重复的添加所有的N-8-edge
                if (p.x > 0)
                {
                    //float w = leftW.at<float>(p);
                    graph.setEdges(vtxIdx, vtxIdx - 1, leftW.at<float>(p), leftW.at<float>(p), edgenum);
                    //leftEdge[vtxIdx] = edgenum; //获得与左边相连接的边的编号
                }
                if (p.x > 0 && p.y > 0)
                {
                    //float w = upleftW.at<float>(p);
                    graph.setEdges(vtxIdx, vtxIdx - cols - 1, upleftW.at<float>(p), upleftW.at<float>(p), edgenum);
                    //upleftEdge[vtxIdx] = edgenum; //获得与左上边相连接的边的编号，这里如有需要可以优化
                }
                if (p.y > 0)
                {
                    //float w = upW.at<float>(p);
                    graph.setEdges(vtxIdx, vtxIdx - cols, upW.at<float>(p), upW.at<float>(p), edgenum);
                    //upEdge[vtxIdx] = edgenum; //获得与上边相连接的边的编号
                }
                if (p.x < cols - 1 && p.y > 0)
                {
                    //float w = uprightW.at<float>(p);
                    graph.setEdges(vtxIdx, vtxIdx - cols + 1, uprightW.at<float>(p), uprightW.at<float>(p), edgenum);
                    //uprightEdge[vtxIdx] = edgenum; //获得与右上边相连接的边的编号
                }
                vtxIdx++; //节点对象索引
            }
        }
    }
    void estimateSegmentation(GCGraphMy<float> &graph) //最大流最小割分割图像,输出结果
    {
        graph.maxFlow();
    }

private:
    float calFgdPrioriCost(uchar &color) //计算前景颜色在直方图中出现的概率
    {
        float p = FgPHist[color];
        return p;
    }
    float calBgdPrioriCost(uchar &color) //计算背景颜色在直方图中出现的概率
    {
        float p = BgPHist[color];
        return p;
    }
};

class graphCut
{
public:
    int rows;
    int cols;
    GCApplication *gcapp;
    GCGraphMy<float> stGraphMy; //图对象

    void generateGraphCutImg(PTCloud *orderpt, char* imgname)
    {
        cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
        for (int l = 0; l < rows; l++)
            for (int c = 0; c < cols; c++)
            {
                if(orderpt->mPTCloudInfoMap[l][c]->mergeMarkType==0)
                {
                    img.at<cv::Vec3b>(l, c) = cv::Vec3b(255, 255, 255);
                    //std::cout<<"error"<<std::endl;
                    continue;
                }
                if (MASK_BG_COLOR == orderpt->mPTCloudInfoMap[l][c]->backOrFore)
                {
                    img.at<cv::Vec3b>(l, c) = cv::Vec3b(114, 204, 14);
                }
                else if(MASK_FG_COLOR == orderpt->mPTCloudInfoMap[l][c]->backOrFore)
                {
                    img.at<cv::Vec3b>(l, c) = cv::Vec3b(67, 67, 250);
                }
                else
                {
                    img.at<cv::Vec3b>(l, c) = cv::Vec3b(250, 67, 67);
                }
            }
        cv::flip(img, img, 0);
        cv::imwrite(imgname, img);
    }

    void generateGraphCutImgResult(PTCloud *orderpt, char* imgname)
    {
        cv::Mat img(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
        for (int l = 0; l < rows; l++)
            for (int c = 0; c < cols; c++)
            {
                if(orderpt->mPTCloudInfoMap[l][c]->mergeMarkType==0)
                {
                    img.at<cv::Vec3b>(l, c) = cv::Vec3b(255, 255, 255);
                    //std::cout<<"error"<<std::endl;
                    continue;
                }
                if (10 == orderpt->mPTCloudInfoMap[l][c]->setType)
                {
                    img.at<cv::Vec3b>(l, c) = cv::Vec3b(114, 204, 14);
                }
                else
                {
                    img.at<cv::Vec3b>(l, c) = cv::Vec3b(67, 67, 250);
                }
            }
        cv::flip(img, img, 0);
        cv::imwrite(imgname, img);
    }


    long groundCut(PTCloud *orderpt, int useproperties)
    {
        auto start = std::chrono::system_clock::now();

        // timer mTimergraphConstruct;
        // mTimergraphConstruct.start();
        gcapp->graphConstruct(orderpt, stGraphMy, useproperties); //创建图像结构和连接关系
        // mTimergraphConstruct.stop();
        // mTimergraphConstruct.print("图像结构构建总耗时：");

        //打印出图割所需要的时间
        //timer mTimergraphcut;
        //mTimergraphcut.start();
        gcapp->estimateSegmentation(stGraphMy);                                    //最大流最小割分割点云并在图像中标记结果
        //mTimergraphcut.stop();
        //mTimergraphcut.print("图割求解耗时：");

//        //Mat img(LINE,CIRCLEMAXLEN,CV_8UC3,Scalar(0,0,0));
//
        for (int l = 0; l < rows; l++)
            for (int c = 0; c < cols; c++)
            {
                if(orderpt->mPTCloudInfoMap[l][c]->mergeMarkType == 0)
                    continue;

                if (1 == stGraphMy.inSourceSegment(l * cols + c))
                {
                    orderpt->mPTCloudInfoMap[l][c]->setType = 20;
                }
                else
                {
                    orderpt->mPTCloudInfoMap[l][c]->setType = 10;
                }
            }

        // imshow("",img);
        // waitKey(10);
        stGraphMy.releasedata();
        auto end = std::chrono::system_clock::now();
        int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        return time;
    }
    graphCut(unsigned int grows, unsigned int gcols)
    {
        rows = grows;
        cols = gcols;

        gcapp = new GCApplication(rows, cols); //初始化图对象

        unsigned int vtxCount = rows * cols;
        //最外乘2为正反向边，4为左，左上，上，上右四个点，减去边界不存在的点，则得到总的边数(不包括源点和汇点连接的那些边)
        unsigned int edgeCount = 2 * (4 * cols * rows - (3 * (cols + rows) - 2));

        stGraphMy.create(vtxCount, edgeCount); //开辟内存

        for (int l = 0; l < rows; l++)
            for (int c = 0; c < cols; c++)
            {
                //图顶点添加
                stGraphMy.addVtx();
                //图边添加
                if (c > 0)
                {
                    stGraphMy.addEdges();
                }
                if (c > 0 && l > 0)
                {
                    stGraphMy.addEdges();
                }
                if (l > 0)
                {
                    stGraphMy.addEdges();
                }
                if (c < cols - 1 && l > 0)
                {
                    stGraphMy.addEdges();
                }
            }
    }
};