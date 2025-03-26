This project is the code for my paper published in RAL, "A Fast Point Cloud Ground Segmentation Approach Based on Block-Sparsely Connected Coarse-to-Fine Markov Random Field. " It is provided for interested researchers to reproduce the article's results.

---

# Running CTFMRFv2
Before running this project, ensure you have **ROS 2** installed.
#### Clone the Repository
```Bash
git clone https://github.com/weixinhum2023/CTFMRFv2
```
#### Dataset Download

To accelerate runtime performance, the dataset has been preprocessed and is available for download at the following link. If needed, a corresponding dataset format conversion tool will be provided in the future.

Download Preprocessed Dataset: 
https://pan.baidu.com/s/1iUQLHQC-Jc8mgBUhV4Yhxg?pwd=uc57

#### ​Dataset Setup Instructions
After downloading the dataset, configure its path by modifying the rootPath variable in:
tool/Settings.hpp

#### Build & Run
```Bash
# ​Terminal 1
cd CTFMRFv2/
colcon build
source ./install/setup.zsh #zsh terminal
source ./install/setup.bash #bash terminal
ros2 run ctfmrfv2 CTFMRFv2
# ​Terminal 2
rviz2 
```

# Citing CTFMRF
If you use CTFMRF in your research, please cite the papers:

>ARTICLE{CTFMRFv2,  
  author={Huang, Weixin and Lin, Linglong and Wang, Shaobo and Fan, Zhun and Yu, Biao and Chen, Jiajia},  
  journal={IEEE Robotics and Automation Letters},   
  title={A Fast Point Cloud Ground Segmentation Approach Based on Block-Sparsely Connected Coarse-to-Fine Markov Random Field},   
  year={2025},  
  volume={10},  
  number={4},  
  pages={3843-3850},
  doi={10.1109/LRA.2025.3546071}}

>@ARTICLE{CTFMRFv1,  
  author={Huang, Weixin and Liang, Huawei and Lin, Linglong and Wang, Zhiling and Wang, Shaobo and Yu, Biao and Niu, Runxin},  
  journal={IEEE Transactions on Intelligent Transportation Systems},   
  title={A Fast Point Cloud Ground Segmentation Approach Based on   Coarse-To-Fine Markov Random Field},   
  year={2022},  
  volume={23},  
  number={7},  
  pages={7841-7854},  
  doi={10.1109/TITS.2021.3073151}}

---