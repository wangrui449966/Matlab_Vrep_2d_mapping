# Matlab_Vrep_2d_mapping
Use Matlab to simulate 2D mapping with Vrep 使用Matlab和Vrep进行2D建图仿真



各个文件夹里是啥：

matlab_files文件夹中是matlab工程文件

vrep_ttt文件夹中是vrep场景文件

py_control文件夹中是python写的用于遥控小车的遥控器，运行main.py即可（需要安装pygame）(pip install pygame)




怎么用：

分别用Vrep、Matlab（我用的是R2019a 64-bit）、python（建议使用pyCharm）打开三个工程以后，

先运行Vrep仿真，然后运行Matlab和python工程，选中python工程的窗口，按键盘上的上下左右键来控制小车的运动，同时matlab会进行动态建图。

注：python需要安装pygame包（pip install pygame）
