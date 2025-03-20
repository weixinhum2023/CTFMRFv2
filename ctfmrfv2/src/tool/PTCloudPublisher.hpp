/********************************
作者：Weixinhuang
日期：20220112
功能：Publish the point cloud for display in rviz2.
********************************/

#pragma once

#include <cstdio>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <rclcpp/rclcpp.hpp>
#include "PTCloudDataFrame.hpp"

using namespace std::chrono_literals;

class PTCloudPublisher : public rclcpp::Node
{
public:
    typeMap mTypeMap; // Category coloring
    PTCloud *mPTCloud = nullptr; // Point cloud object
    explicit PTCloudPublisher(PTCloud *getPTCloud) : Node("publisher")
    {
        mPTCloud = getPTCloud;
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("topic", 10);
        counter = 0;
    }

public:
    void doPTCloudPublish()
    {
        auto pointCloud = sensor_msgs::msg::PointCloud();

        pointCloud.header.frame_id = "map";
        pointCloud.points.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels.resize(13); 
        pointCloud.channels[0].name = "rgb";
        pointCloud.channels[0].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[1].name = "line";
        pointCloud.channels[1].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[2].name = "heightDif";
        pointCloud.channels[2].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[3].name = "theoreticalHeightDif";
        pointCloud.channels[3].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[4].name = "distance";
        pointCloud.channels[4].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[5].name = "lowestPT";
        pointCloud.channels[5].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[6].name = "cols";
        pointCloud.channels[6].values.resize(mPTCloud->cols * mPTCloud->rows);

        pointCloud.channels[7].name = "grid_x";
        pointCloud.channels[7].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[8].name = "grid_y";
        pointCloud.channels[8].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[9].name = "grid_lowest_line";
        pointCloud.channels[9].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[10].name = "grid_lowest_cols";
        pointCloud.channels[10].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[11].name = "grid_heightest_line";
        pointCloud.channels[11].values.resize(mPTCloud->cols * mPTCloud->rows);
        pointCloud.channels[12].name = "grid_heightest_cols";
        pointCloud.channels[12].values.resize(mPTCloud->cols * mPTCloud->rows);
        

        int ptNum = 0;
        for (int l = 0; l < mPTCloud->rows; l++)
        {
            for (int c = 0; c < mPTCloud->cols; c++)
            {
                if(mPTCloud->mPTCloudInfo[l][c].mergeMarkType==0)
                    continue;
                
                pointCloud.points[ptNum].x = mPTCloud->mPTCloud[l][c].x;
                pointCloud.points[ptNum].y = mPTCloud->mPTCloud[l][c].y;
                pointCloud.points[ptNum].z = mPTCloud->mPTCloud[l][c].z;
            
                unsigned char r = mTypeMap.type_map[mPTCloud->mPTCloudInfo[l][c].setType][0];
                unsigned char g = mTypeMap.type_map[mPTCloud->mPTCloudInfo[l][c].setType][1];
                unsigned char b = mTypeMap.type_map[mPTCloud->mPTCloudInfo[l][c].setType][2];

#if MARKTP_TN_FN_FP
                if(mPTCloud->mPTCloudInfo[l][c].setType==10 && mPTCloud->mPTCloudInfo[l][c].mergeMarkType==20)//标记为地面实际上是障碍物点，过分割，FN
                {
                    r = 14;
                    g = 14;
                    b = 14;
                }
                else if(mPTCloud->mPTCloudInfo[l][c].setType==20 && mPTCloud->mPTCloudInfo[l][c].mergeMarkType==10)//标记为障碍物点实际上是地面，过分割，FP
                {
                    // 普通障碍物
                    r = 14;
                    g = 114;
                    b = 204;
                }
#endif
                
                uint32_t rgb = r << 16 | g << 8 | b; // rgb
                float rgb_float;
                memcpy(&rgb_float, &rgb, sizeof(float));
                pointCloud.channels[0].values[ptNum] = rgb_float;

                pointCloud.channels[1].values[ptNum] = (float)l;

                pointCloud.channels[2].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].heightDif;            
                pointCloud.channels[3].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].theoreticalHeightDif; 
                pointCloud.channels[4].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].distance;             
                pointCloud.channels[5].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].lowestPt;             
                pointCloud.channels[6].values[ptNum] = c;                                               

                pointCloud.channels[7].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].gridx;                                                 // 所在列的位置
                pointCloud.channels[8].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].gridy;                                                 // 所在列的位置

                pointCloud.channels[9].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].lowest_line;                                                 // 所在列的位置
                pointCloud.channels[10].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].lowest_cols;                                                 // 所在列的位置

                pointCloud.channels[11].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].heightest_line;                                                 // 所在列的位置
                pointCloud.channels[12].values[ptNum] = mPTCloud->mPTCloudInfo[l][c].heightest_cols;                                                 // 所在列的位置

                ptNum++;
            }
        }
        counter++;
        publisher_->publish(pointCloud);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
    int counter;
};
