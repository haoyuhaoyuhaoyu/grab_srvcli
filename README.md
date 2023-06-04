# grab_srvcli
功能说明：基于睿尔曼机器臂与因时灵巧手进行开发，通过键盘控制，可实现机器臂在三维空间的运动，并用灵巧手抓取物体。
环境配置：
Ubuntu22.04 + Ros2 humble
Ros2安装参考https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#
使用方法：
该程序基于Ros2 Service，使用时需启动grab_server与grab_client，其中grab_server通过ros2 launch启动。Ros2 Service启动与使用，参考
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html
代码说明：
1.grab_client为指令发送端，通过扫描键盘输入，发送至服务端。
2.grab_server为服务端，接收发送端指令，执行相应的操作。对机器臂的控制均为角度透传方式，调用睿尔曼厂商自带API，IK解算使用trac_ik进行解算，其使用方法参考
https://traclabs.com/projects/trac-ik。 该程序中的Get_pos线程用于获取摄像头数据，得到二维码在机器人坐标系中的位置。
