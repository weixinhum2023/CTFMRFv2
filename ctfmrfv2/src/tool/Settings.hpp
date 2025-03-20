#pragma once
#include <string>
std::string labelMapPath="./src/toolset/labelMap.txt";
std::string rootPath="/home/weixinhum/第二篇文章使用点云数据集";

#define STOP 0
#define STOPNUM 546
#define RUNSEQ 0 //When set to 0, run all sequences.

#define MARKTP_TN_FN_FP 0 //When set to 0, only show ground points and non-ground points. If set to 1, display TP (True Positives), TN (True Negatives), FN (False Negatives), and FP (False Positives).

#define evaluations_40_8 0 //evaluations within 8 m of the vehicle’s horizontal coordinate and a total distance between 40 and 50 m


//SEMANTICKITTI////////
#define M_PT_ROW 64
#define M_PT_COL 1800
#define SEQ_NUM 11