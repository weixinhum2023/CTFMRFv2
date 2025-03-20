#include <rclcpp/rclcpp.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>  // 必须包含此头文件

#include "tool/Settings.hpp"
#include "tool/PTCloudDataFrame.hpp"
#include "tool/PTCloudRead.hpp"
#include "tool/PTCloudPublisher.hpp"
#include "tool/PTCloudGSegEvaluation.hpp"
#include "tool/timer.hpp"

#include "ringGridHDiffNew.hpp"
#include "spatiotemporalAdjacentPoints.hpp"
#include "GraphCut.hpp"
#include "groundpointssearch.hpp"

void graphcut(graphCut *cut, PTCloud *PTCloud)
{
    cut->groundCut(PTCloud, 0);
}

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    // 初始化ROS2对象
    rclcpp::init(argc, argv);
    // 点云对象创建
    auto *mPTCloud = new PTCloud(M_PT_ROW, M_PT_COL);
    auto *mPTCloud_tl = new PTCloud(M_PT_ROW / 2, M_PT_COL / 2); // 子点云对象初始化
    auto *mPTCloud_tr = new PTCloud(M_PT_ROW / 2, M_PT_COL / 2); // 子点云对象初始化
    auto *mPTCloud_bl = new PTCloud(M_PT_ROW / 2, M_PT_COL / 2); // 子点云对象初始化
    auto *mPTCloud_br = new PTCloud(M_PT_ROW / 2, M_PT_COL / 2); // 子点云对象初始化
    // 将点云对象和点云地图建立映射关系
    for (int l = 0; l < M_PT_ROW; l++)
        for (int c = 0; c < M_PT_COL; c++)
        {
            mPTCloud->mPTCloudMap[l][c] = &mPTCloud->mPTCloud[l][c];
            mPTCloud->mPTCloudInfoMap[l][c] = &mPTCloud->mPTCloudInfo[l][c];
        }
    for (int l = 0; l < M_PT_ROW / 2; l++)
        for (int c = 0; c < M_PT_COL / 2; c++)
        {
            mPTCloud_tl->mPTCloudMap[l][c] = &mPTCloud->mPTCloud[l * 2][c * 2];
            mPTCloud_tl->mPTCloudInfoMap[l][c] = &mPTCloud->mPTCloudInfo[l * 2][c * 2];

            mPTCloud_tr->mPTCloudMap[l][c] = &mPTCloud->mPTCloud[l * 2][c * 2 + 1];
            mPTCloud_tr->mPTCloudInfoMap[l][c] = &mPTCloud->mPTCloudInfo[l * 2][c * 2 + 1];

            mPTCloud_bl->mPTCloudMap[l][c] = &mPTCloud->mPTCloud[l * 2 + 1][c * 2];
            mPTCloud_bl->mPTCloudInfoMap[l][c] = &mPTCloud->mPTCloudInfo[l * 2 + 1][c * 2];

            mPTCloud_br->mPTCloudMap[l][c] = &mPTCloud->mPTCloud[l * 2 + 1][c * 2 + 1];
            mPTCloud_br->mPTCloudInfoMap[l][c] = &mPTCloud->mPTCloudInfo[l * 2 + 1][c * 2 + 1];
        }

    graphCut mgraphCut(mPTCloud->rows, mPTCloud->cols);
    graphCut mgraphCut_tl(mPTCloud->rows / 2, mPTCloud->cols / 2);
    graphCut mgraphCut_tr(mPTCloud->rows / 2, mPTCloud->cols / 2);
    graphCut mgraphCut_bl(mPTCloud->rows / 2, mPTCloud->cols / 2);
    graphCut mgraphCut_br(mPTCloud->rows / 2, mPTCloud->cols / 2);
    // 点云发布对象创建
    auto *mPTCloudPublisher = new PTCloudPublisher(mPTCloud);
    PTCloudGSegEvaluation mPTCloudGSegEvaluation(mPTCloud);
    timer mTotalTimer; // 总计时器
    // 创建地面分割对象
    ringGridHDiffNew mRingGridHDiff(mPTCloud);                            // 新一代环状栅格高度差
    SpatioTemporalAdjacentPoints mSpatioTemporalAdjacentPoints(mPTCloud); // 时空临近算法
    groundPointsSearch mGroundPointsSearch(mPTCloud);

    mTotalTimer.start();
    for (int readSeq = RUNSEQ; rclcpp::ok() && readSeq < SEQ_NUM; readSeq++) // 读取全部数据
    {
        PTCloudRead mPTCloudRead(mPTCloud, readSeq);
        mPTCloudGSegEvaluation.init();
        unsigned int seq, seqPTCloudNum;
        
        while (rclcpp::ok() && mPTCloudRead.readPTCloud(seq, seqPTCloudNum))
        {
            mPTCloud->dataPreprocessing(); // 数据预处理
            mPTCloudGSegEvaluation.start(seq, seqPTCloudNum);

            //?环状栅格地面拟合
            mRingGridHDiff.run();//Baseline
            mSpatioTemporalAdjacentPoints.run();
            mGroundPointsSearch.run();
            if (mPTCloud->rows != 32)
            {
                ////////////////////图割-多线程////////////////////
                auto fut1 = std::async(std::launch::async, graphcut, &mgraphCut_tl, mPTCloud_tl);
                auto fut2 = std::async(std::launch::async, graphcut, &mgraphCut_tr, mPTCloud_tr);
                auto fut3 = std::async(std::launch::async, graphcut, &mgraphCut_bl, mPTCloud_bl);
                auto fut4 = std::async(std::launch::async, graphcut, &mgraphCut_br, mPTCloud_br);
                fut1.get();
                fut2.get();
                fut3.get();
                fut4.get();
            }
            else
            {
                graphcut(&mgraphCut, mPTCloud);
            }

            mPTCloudGSegEvaluation.stop();
            mPTCloudPublisher->doPTCloudPublish();                      // 点云发布
        }
        mPTCloudGSegEvaluation.printSeqEvaluate(readSeq); // 把评估完的系列结果打印出来
    }
    mPTCloudGSegEvaluation.saveEvaluateResultToCSV(); // 将评估结果存入文件中
    mTotalTimer.stop();
    mTotalTimer.print("总耗时：",true);
    std::cout << std::endl
    << "数据处理完成hhh" << std::endl;
    delete mPTCloud;
    delete mPTCloud_tl;
    delete mPTCloud_tr;
    delete mPTCloud_bl;
    delete mPTCloud_br;
    delete mPTCloudPublisher;
    return 0;
}
