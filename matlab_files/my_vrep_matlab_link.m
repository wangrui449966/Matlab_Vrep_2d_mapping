%2020-6-12    my_vrep_matlab_link
%实现类调用，更加方便，并节省调用


classdef my_vrep_matlab_link
    
    properties %类属性
        name;%名字，构造类实体的时候被赋值
        
    end
    
    methods
        %构造实例，参数是给实例起的名字
        function obj = my_vrep_matlab_link(name)
            obj.name=name;
        end
        
        %初始化vrep与matlab的连接，参数是remApi实体
        function s_vm_ret=vrep_matlab_link_init(obj,c_vrep,s_vm_in)
            c_vrep.simxFinish(-1); % just in case, close all opened connections

            %打开端口并记录ID
            s_vm_in.dr20ID=c_vrep.simxStart('127.0.0.1',66666,true,true,5000,5);%66666是小车的端口

            %检查是否打开成功
            if (s_vm_in.dr20ID<=-1)
                fprintf('DR20ID获取失败');
                while true
                end
            end

            fprintf('link成功，等0.2秒，确保vrep准备就绪\n');
            pause(0.2);

            %获取车体句柄（用来读取 位 姿）（这里获取的实际上是雷达的句柄，因为雷达在轮轴正上方）
            [ret,s_vm_in.dr20_body_handle]=c_vrep.simxGetObjectHandle(s_vm_in.dr20ID,'Hokuyo_URG_04LX_UG01',c_vrep.simx_opmode_blocking);
            if(ret~=0)
                while true
                    fprintf('dr20车体句柄获取失败\n')
                end
            end

            %获取轮子的句柄（用来输出速度）
            [ret,s_vm_in.dr20_left_wheel_handle]=c_vrep.simxGetObjectHandle(s_vm_in.dr20ID,'dr20_leftWheelJoint_',c_vrep.simx_opmode_blocking);
            if (ret~=0)
                while true
                    fprintf('dr20左轮句柄获取失败\n');
                end
            end
            [ret,s_vm_in.dr20_right_wheel_handle]=c_vrep.simxGetObjectHandle(s_vm_in.dr20ID,'dr20_rightWheelJoint_',c_vrep.simx_opmode_blocking);
            if (ret~=0)
                while true
                    fprintf('dr20左轮句柄获取失败\n');
                end
            end





            % Now retrieve streaming data (i.e. in a non-blocking fashion):
            fprintf('vrep_matlab_simulation_start!\n');
            c_vrep.simxAddStatusbarMessage(s_vm_in.dr20ID,'Matlab linked!',c_vrep.simx_opmode_oneshot);%在vrep里输出一行matlab已经连接

            %第一次调用必须是streaming模式（手册里说的）
            c_vrep.simxGetObjectOrientation(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,-1,c_vrep.simx_opmode_streaming);
            c_vrep.simxGetObjectPosition(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,-1,c_vrep.simx_opmode_streaming);
            c_vrep.simxGetObjectVelocity(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,c_vrep.simx_opmode_streaming);
            [retCode,lidar_pack]=c_vrep.simxGetStringSignal(s_vm_in.dr20ID,'UG01_distance',c_vrep.simx_opmode_streaming );%for first call，手册里写第一次调用要streaming
            %先确保正确读取了一次雷达，防止后面出错
            while (retCode~=c_vrep.simx_return_ok)
                [retCode,lidar_pack]=c_vrep.simxGetStringSignal(s_vm_in.dr20ID,'UG01_distance',c_vrep.simx_opmode_buffer);
            end
            s_vm_in.lidar_array_raw=c_vrep.simxUnpackFloats(lidar_pack);
            fprintf('雷达读取成功，开始进入仿真\n')
            s_vm_ret=s_vm_in;%更新结构体
        end%function vrep_matlab_link_init(c_vrep)
        
        function s_vm_ret=wheel_velocity_set(obj,c_vrep,s_vm_in,which_wheel,speed)
            %设置轮子速度，which_wheel选择轮子的字符，'L'为左轮，'R'为右轮，不区分大小写
            %speed 为速度
            if(which_wheel=='l' || which_wheel=='L')%左
                c_vrep.simxSetJointTargetVelocity(s_vm_in.dr20ID,s_vm_in.dr20_left_wheel_handle,speed,c_vrep.simx_opmode_streaming);
            end
            if(which_wheel=='r' || which_wheel=='R')%右
                c_vrep.simxSetJointTargetVelocity(s_vm_in.dr20ID,s_vm_in.dr20_right_wheel_handle,speed,c_vrep.simx_opmode_streaming);
            end
            s_vm_ret=s_vm_in;
        end
        
        function s_vm_ret=lidar_array_update(obj,c_vrep,s_vm_in)
            [retCode,lidar_pack]=c_vrep.simxGetStringSignal(s_vm_in.dr20ID,'UG01_distance',c_vrep.simx_opmode_buffer);
            if (retCode==c_vrep.simx_return_ok)%==0 有更新
                s_vm_in.lidar_array_raw=c_vrep.simxUnpackFloats(lidar_pack);
            else%没有更新
                s_vm_in.lidar_array_raw=current_lidar_array;
            end
            s_vm_ret=s_vm_in;
        end
        
        function [s_vm_ret,distance]=lidar_point_read(obj,s_vm_in,index)
            distance=s_vm_in.lidar_array_raw(index*3-1);
            s_vm_ret=s_vm_in;
        end
        
        %更新车体位姿（实际上是雷达位姿）
        function [s_vm_ret]=car_pose_update(obj,c_vrep,s_vm_in)
            %先获取z轴euler
            [retCode,eulerAngles_vec3]=c_vrep.simxGetObjectOrientation(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,-1,c_vrep.simx_opmode_buffer);
            if(retCode~=0)%车头角度获取失败
                fprintf('angle_z get failed!!!\n');
            else
                s_vm_in.angle_z=eulerAngles_vec3(3);
            end

            %再获取2d坐标
            [retCode,position3d]=c_vrep.simxGetObjectPosition(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,-1,c_vrep.simx_opmode_buffer);
            if(retCode~=0)
                fprintf('car_body_pos get failed!!!\n');
            else
                s_vm_in.pos_x=position3d(1);
                s_vm_in.pos_y=position3d(2);
            end
            
            s_vm_ret=s_vm_in;

        end
        
        %更新车体速度
        function [s_vm_ret]=car_velocity_update(obj,c_vrep,s_vm_in)
            [retCode,linearV,angularV]=c_vrep.simxGetObjectVelocity(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,c_vrep.simx_opmode_buffer);
            if(retCode~=0)
                while true
                    fprintf("Vrep velocity get failed!!!\n");
                end
            end
            s_vm_in.Vx=linearV(1);
            s_vm_in.Vy=linearV(2);
            s_vm_in.dAngularZ=angularV(3);
            s_vm_ret=s_vm_in;
        end

        %读取障碍物
        function [s_vm_ret,yes_it_is]=is_obstacle_80cmHighWall200cm(obj,vrep,s_vm_in,pos_x,pos_y,pos_z)
            persistent is_inited;%静态变量，是否已经初始化好了，第一次调用得初始化一下，其实就是第一次的读取方式不一样
            persistent handle_200cm;
            persistent handle_200cm0;
            persistent handle_200cm1;
            persistent handle_200cm2;
            persistent handle_200cm3;
            persistent handle_200cm4;
            persistent handle_200cm5;
            persistent handle_200cm6;
            
            s_vm_ret=s_vm_in;

            RtoA=1.0/3.1415926535*180.0;%弧度转角度系数
            Wall_x_length=1.5001/10.0;%x方向长度
            Wall_y_length=2.0000;%y方向长度
            Wall_z_length=8.0/10.0;%z方向长度 即高度

            if isempty(is_inited)
                is_inited=false;
            end

            if(is_inited==false)%第一次读取
                is_inited=true;%下次就不是第一次了
                read_mode=vrep.simx_opmode_streaming;
                [retCode,handle_200cm]=vrep.simxGetObjectHandle(s_vm_in.dr20ID,'80cmHighWall200cm',vrep.simx_opmode_blocking);
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm,-1,read_mode);
                [retCode,handle_200cm0]=vrep.simxGetObjectHandle(s_vm_in.dr20ID,'80cmHighWall200cm0',vrep.simx_opmode_blocking);
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm0,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm0,-1,read_mode);
                [retCode,handle_200cm1]=vrep.simxGetObjectHandle(s_vm_in.dr20ID,'80cmHighWall200cm1',vrep.simx_opmode_blocking);
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm1,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm1,-1,read_mode);
                [retCode,handle_200cm2]=vrep.simxGetObjectHandle(s_vm_in.dr20ID,'80cmHighWall200cm2',vrep.simx_opmode_blocking);
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm2,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm2,-1,read_mode);
                [retCode,handle_200cm3]=vrep.simxGetObjectHandle(s_vm_in.dr20ID,'80cmHighWall200cm3',vrep.simx_opmode_blocking);
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm3,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm3,-1,read_mode);
                [retCode,handle_200cm4]=vrep.simxGetObjectHandle(s_vm_in.dr20ID,'80cmHighWall200cm4',vrep.simx_opmode_blocking);
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm4,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm4,-1,read_mode);
                [retCode,handle_200cm5]=vrep.simxGetObjectHandle(s_vm_in.dr20ID,'80cmHighWall200cm5',vrep.simx_opmode_blocking);
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm5,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm5,-1,read_mode);
                [retCode,handle_200cm6]=vrep.simxGetObjectHandle(s_vm_in.dr20ID,'80cmHighWall200cm6',vrep.simx_opmode_blocking);
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm6,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm6,-1,read_mode);
            end

            read_mode=vrep.simx_opmode_streaming;%这里依然是streaming，不然有时读不出来
            %read_mode=vrep.simx_opmode_buffer;


            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%被转了90度的墙（地图里横向）
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%横向墙
                x_min=wall_pos(1)-Wall_x_length/2.0;
                x_max=wall_pos(1)+Wall_x_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_y_length/2.0;
                y_max=wall_pos(2)+Wall_y_length/2.0;
            end
            if(pos_x>=x_min&&pos_x<=x_max&&...
                pos_y>=y_min&&pos_y<=y_max&&...
                pos_z>=z_min&&pos_z<=z_max)
                yes_it_is=true;
                return;
            end


            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm0,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm0,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%被转了90度的墙（地图里横向）
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%横向墙
                x_min=wall_pos(1)-Wall_x_length/2.0;
                x_max=wall_pos(1)+Wall_x_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_y_length/2.0;
                y_max=wall_pos(2)+Wall_y_length/2.0;
            end
            if(pos_x>=x_min&&pos_x<=x_max&&...
                pos_y>=y_min&&pos_y<=y_max&&...
                pos_z>=z_min&&pos_z<=z_max)
                yes_it_is=true;
                return;
            end


            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm1,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm1,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%被转了90度的墙（地图里横向）
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%横向墙
                x_min=wall_pos(1)-Wall_x_length/2.0;
                x_max=wall_pos(1)+Wall_x_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_y_length/2.0;
                y_max=wall_pos(2)+Wall_y_length/2.0;
            end
            if(pos_x>=x_min&&pos_x<=x_max&&...
                pos_y>=y_min&&pos_y<=y_max&&...
                pos_z>=z_min&&pos_z<=z_max)
                yes_it_is=true;
                return;
            end


            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm2,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm2,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%被转了90度的墙（地图里横向）
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%横向墙
                x_min=wall_pos(1)-Wall_x_length/2.0;
                x_max=wall_pos(1)+Wall_x_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_y_length/2.0;
                y_max=wall_pos(2)+Wall_y_length/2.0;
            end
            if(pos_x>=x_min&&pos_x<=x_max&&...
                pos_y>=y_min&&pos_y<=y_max&&...
                pos_z>=z_min&&pos_z<=z_max)
                yes_it_is=true;
                return;
            end

            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm3,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm3,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%被转了90度的墙（地图里横向）
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%横向墙
                x_min=wall_pos(1)-Wall_x_length/2.0;
                x_max=wall_pos(1)+Wall_x_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_y_length/2.0;
                y_max=wall_pos(2)+Wall_y_length/2.0;
            end
            if(pos_x>=x_min&&pos_x<=x_max&&...
                pos_y>=y_min&&pos_y<=y_max&&...
                pos_z>=z_min&&pos_z<=z_max)
                yes_it_is=true;
                return;
            end



            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm4,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm4,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%被转了90度的墙（地图里横向）
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%横向墙
                x_min=wall_pos(1)-Wall_x_length/2.0;
                x_max=wall_pos(1)+Wall_x_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_y_length/2.0;
                y_max=wall_pos(2)+Wall_y_length/2.0;
            end
            if(pos_x>=x_min&&pos_x<=x_max&&...
                pos_y>=y_min&&pos_y<=y_max&&...
                pos_z>=z_min&&pos_z<=z_max)
                yes_it_is=true;
                return;
            end


            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm5,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm5,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%被转了90度的墙（地图里横向）
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%横向墙
                x_min=wall_pos(1)-Wall_x_length/2.0;
                x_max=wall_pos(1)+Wall_x_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_y_length/2.0;
                y_max=wall_pos(2)+Wall_y_length/2.0;
            end
            if(pos_x>=x_min&&pos_x<=x_max&&...
                pos_y>=y_min&&pos_y<=y_max&&...
                pos_z>=z_min&&pos_z<=z_max)
                yes_it_is=true;
                return;
            end

            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm6,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm6,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%被转了90度的墙（地图里横向）
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%横向墙
                x_min=wall_pos(1)-Wall_x_length/2.0;
                x_max=wall_pos(1)+Wall_x_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_y_length/2.0;
                y_max=wall_pos(2)+Wall_y_length/2.0;
            end
            if(pos_x>=x_min&&pos_x<=x_max&&...
                pos_y>=y_min&&pos_y<=y_max&&...
                pos_z>=z_min&&pos_z<=z_max)
                yes_it_is=true;
                return;
            end

            yes_it_is=false;%不在任何的墙壁中，这里是空气（或者是小车自己）
            s_vm_ret=s_vm_in;
        end
        
        %根据雷达与位姿信息动态建图
        function [s_vm_ret]=dynamic_mapping(obj,s_vm_in)
            AtoR=1.0/180.0*3.1415926535;
            dilate_neighborhood_mode=0;%膨胀模式，0不膨胀，1四邻域，2八邻域
            pixel_obstacle_value=127;%如果检测到障碍物，则这个像素是这个值
            for i=1:684
                [s_vm_in,distance]=obj.lidar_point_read(s_vm_in,i);%读取这个点的距离
                if(distance>0)%这个点的数据有效（超出量程则是0）
                    absolute_angle=AtoR*((i-1)*240/(684-1)+(-120))+s_vm_in.angle_z;%算出这个雷达点的绝对角度（弧度）（与车体朝向无关）
                    lidar_pose_x=s_vm_in.pos_x;
                    lidar_pose_y=s_vm_in.pos_y;
                    obstacle_x=lidar_pose_x+distance*cos(absolute_angle-0*AtoR);
                    obstacle_y=lidar_pose_y+distance*cos(absolute_angle-90*AtoR);
                    pixel_x=round(obstacle_x*100.0);%像素精度1cm，四舍五入取整
                    pixel_y=round(obstacle_y*100.0);
                    if dilate_neighborhood_mode==0%不膨胀
                        if(pixel_x>0 && pixel_x<=500 && pixel_y>0 &&pixel_y<=500)
                            s_vm_in.map(500-pixel_y,pixel_x)=pixel_obstacle_value;
                        end
                    elseif dilate_neighborhood_mode==1%四邻域膨胀
                        if(pixel_x>0 && pixel_x<=500 && pixel_y>0 &&pixel_y<=500)
                            s_vm_in.map(pixel_y,pixel_x)=pixel_obstacle_value;
                        end
                        if(pixel_x-1>0 && pixel_x-1<=500 && pixel_y>0 &&pixel_y<=500)
                            s_vm_in.map(pixel_y-1,pixel_x-1)=pixel_obstacle_value;
                        end
                        if(pixel_x+1>0 && pixel_x+1<=500 && pixel_y>0 &&pixel_y<=500)
                            s_vm_in.map(pixel_y,pixel_x+1)=pixel_obstacle_value;
                        end
                        if(pixel_x>0 && pixel_x<=500 && pixel_y-1>0 &&pixel_y-1<=500)
                            s_vm_in.map(pixel_y-1,pixel_x)=pixel_obstacle_value;
                        end
                        if(pixel_x>0 && pixel_x<=500 && pixel_y+1>0 &&pixel_y+1<=500)
                            s_vm_in.map(pixel_y+1,pixel_x)=pixel_obstacle_value;
                        end
                        
                    elseif dilate_neighborhood_mode==2%八邻域膨胀
                        if(pixel_x>0 && pixel_x<=500 && pixel_y>0 &&pixel_y<=500)
                            s_vm_in.map(pixel_y,pixel_x)=pixel_obstacle_value;
                        end
                        if(pixel_x-1>0 && pixel_x-1<=500 && pixel_y>0 &&pixel_y<=500)
                            s_vm_in.map(pixel_y-1,pixel_x-1)=pixel_obstacle_value;
                        end
                        if(pixel_x+1>0 && pixel_x+1<=500 && pixel_y>0 &&pixel_y<=500)
                            s_vm_in.map(pixel_y,pixel_x+1)=pixel_obstacle_value;
                        end
                        if(pixel_x>0 && pixel_x<=500 && pixel_y-1>0 &&pixel_y-1<=500)
                            s_vm_in.map(pixel_y-1,pixel_x)=pixel_obstacle_value;
                        end
                        if(pixel_x>0 && pixel_x<=500 && pixel_y+1>0 &&pixel_y+1<=500)
                            s_vm_in.map(pixel_y+1,pixel_x)=pixel_obstacle_value;
                        end
                        if(pixel_x-1>0 && pixel_x-1<=500 && pixel_y-1>0 &&pixel_y-1<=500)
                            s_vm_in.map(pixel_y-1,pixel_x-1)=pixel_obstacle_value;
                        end
                        if(pixel_x-1>0 && pixel_x-1<=500 && pixel_y+1>0 &&pixel_y+1<=500)
                            s_vm_in.map(pixel_y+1,pixel_x-1)=pixel_obstacle_value;
                        end
                        if(pixel_x+1>0 && pixel_x+1<=500 && pixel_y-1>0 &&pixel_y-1<=500)
                            s_vm_in.map(pixel_y-1,pixel_x+1)=pixel_obstacle_value;
                        end
                        if(pixel_x+1>0 && pixel_x+1<=500 && pixel_y+1>0 &&pixel_y+1<=500)
                            s_vm_in.map(pixel_y+1,pixel_x+1)=pixel_obstacle_value;
                        end
                    end
                end
            end
            s_vm_ret=s_vm_in;
        end
        
        
        
        
  
        
    end%methods
end

