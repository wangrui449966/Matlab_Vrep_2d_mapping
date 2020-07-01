clear;
clc;
close all;

%s_vm:
%s_vm.dr20ID
%s_vm.dr20_body_handle
%s_vm.dr20_left_wheel_handle
%s_vm.dr20_right_wheel_handle
%s_vm.lidar_array_raw
%s_vm.angle_z
%s_vm.pos_x
%s_vm.pos_y
%s_vm.map
%s_vm.Vx
%s_vm.Vy
%s_vm.dAngularZ


%初始化
vrep=remApi('remoteApi'); % vrep remoteApi  remApi类实体
my_vrep_link=my_vrep_matlab_link('Vrep_Matlab_Link');%vrep matlab link，my_vrep_matlab_link类实体

s_vm=[];%创建个变量struct_vrep_matlab，实际是个结构体，方便变量管理
s_vm.map=zeros(500,500,'int8');%500cm * 500cm的地图，精度1cm
s_vm=my_vrep_link.vrep_matlab_link_init(vrep,s_vm);%s_vm自动转为结构体，里面放所有vrep变量，便于变量管理
    
%读取地图
if true
    load('s_vm.mat');
    if true
        imshow(s_vm.map);
    end
end

%动态建图，ctrl+C 退出后 命令行执行一下save s_vm;就可以保存
if false
    while true
        fprintf("dynamic mapping loop\n");
        %更新数据
        %位姿
        s_vm=my_vrep_link.car_pose_update(vrep,s_vm);
        %雷达
        s_vm=my_vrep_link.lidar_array_update(vrep,s_vm);


        %根据雷达与位姿信息动态建图
        [s_vm]=my_vrep_link.dynamic_mapping(s_vm);

        imshow(s_vm.map);


        pause(0.05);
        
    end
end


%类，用法示例
while false
    %设置速度
    %s_vm=my_vrep_link.wheel_velocity_set(vrep,s_vm,'L',-1);
    %s_vm=my_vrep_link.wheel_velocity_set(vrep,s_vm,'R',+1);
    
    %读取雷达
    s_vm=my_vrep_link.lidar_array_update(vrep,s_vm);
    [s_vm,dist]=my_vrep_link.lidar_point_read(s_vm,342);
    [s_vm,dist1]=my_vrep_link.lidar_point_read(s_vm,1);
    [s_vm,dist2]=my_vrep_link.lidar_point_read(s_vm,2);
    [s_vm,dist684]=my_vrep_link.lidar_point_read(s_vm,684);
    fprintf('lidar[342]:%f\n',dist);
    
    %更新车体位姿
    s_vm=my_vrep_link.car_pose_update(vrep,s_vm);
    fprintf("pos_x:%f,pos_y:%f,angular_z:%f\n",s_vm.pos_x,s_vm.pos_y,s_vm.angle_z);
    %更新车体速度
    s_vm=my_vrep_link.car_velocity_update(vrep,s_vm);
    fprintf("Vx:%f,Vy:%f,d_angular_z:%f\n",s_vm.Vx,s_vm.Vy,s_vm.dAngularZ);
    
    %读取地图
    if false
        OBSTACLE_X=[];
        OBSTACLE_Y=[];
        counter=0;
        for i=0.0:0.05:5.0
            for j=0.0:0.05:5.0
                [s_vm,is_obstacle]=my_vrep_link.is_obstacle_80cmHighWall200cm(vrep,s_vm,i,j,0.2);
                if is_obstacle
                    counter=counter+1;
                    OBSTACLE_X(counter)=i;
                    OBSTACLE_Y(counter)=j;
                end
            end
        end
        plot(OBSTACLE_X,OBSTACLE_Y,'o');
        pause(0.1);
    end%在这里打断点可以看到整个地图
    
    pause(0.05);
end
















