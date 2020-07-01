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


%��ʼ��
vrep=remApi('remoteApi'); % vrep remoteApi  remApi��ʵ��
my_vrep_link=my_vrep_matlab_link('Vrep_Matlab_Link');%vrep matlab link��my_vrep_matlab_link��ʵ��

s_vm=[];%����������struct_vrep_matlab��ʵ���Ǹ��ṹ�壬�����������
s_vm.map=zeros(500,500,'int8');%500cm * 500cm�ĵ�ͼ������1cm
s_vm=my_vrep_link.vrep_matlab_link_init(vrep,s_vm);%s_vm�Զ�תΪ�ṹ�壬���������vrep���������ڱ�������
    
%��ȡ��ͼ
if true
    load('s_vm.mat');
    if true
        imshow(s_vm.map);
    end
end

%��̬��ͼ��ctrl+C �˳��� ������ִ��һ��save s_vm;�Ϳ��Ա���
if false
    while true
        fprintf("dynamic mapping loop\n");
        %��������
        %λ��
        s_vm=my_vrep_link.car_pose_update(vrep,s_vm);
        %�״�
        s_vm=my_vrep_link.lidar_array_update(vrep,s_vm);


        %�����״���λ����Ϣ��̬��ͼ
        [s_vm]=my_vrep_link.dynamic_mapping(s_vm);

        imshow(s_vm.map);


        pause(0.05);
        
    end
end


%�࣬�÷�ʾ��
while false
    %�����ٶ�
    %s_vm=my_vrep_link.wheel_velocity_set(vrep,s_vm,'L',-1);
    %s_vm=my_vrep_link.wheel_velocity_set(vrep,s_vm,'R',+1);
    
    %��ȡ�״�
    s_vm=my_vrep_link.lidar_array_update(vrep,s_vm);
    [s_vm,dist]=my_vrep_link.lidar_point_read(s_vm,342);
    [s_vm,dist1]=my_vrep_link.lidar_point_read(s_vm,1);
    [s_vm,dist2]=my_vrep_link.lidar_point_read(s_vm,2);
    [s_vm,dist684]=my_vrep_link.lidar_point_read(s_vm,684);
    fprintf('lidar[342]:%f\n',dist);
    
    %���³���λ��
    s_vm=my_vrep_link.car_pose_update(vrep,s_vm);
    fprintf("pos_x:%f,pos_y:%f,angular_z:%f\n",s_vm.pos_x,s_vm.pos_y,s_vm.angle_z);
    %���³����ٶ�
    s_vm=my_vrep_link.car_velocity_update(vrep,s_vm);
    fprintf("Vx:%f,Vy:%f,d_angular_z:%f\n",s_vm.Vx,s_vm.Vy,s_vm.dAngularZ);
    
    %��ȡ��ͼ
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
    end%�������ϵ���Կ���������ͼ
    
    pause(0.05);
end
















