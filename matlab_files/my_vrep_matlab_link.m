%2020-6-12    my_vrep_matlab_link
%ʵ������ã����ӷ��㣬����ʡ����


classdef my_vrep_matlab_link
    
    properties %������
        name;%���֣�������ʵ���ʱ�򱻸�ֵ
        
    end
    
    methods
        %����ʵ���������Ǹ�ʵ���������
        function obj = my_vrep_matlab_link(name)
            obj.name=name;
        end
        
        %��ʼ��vrep��matlab�����ӣ�������remApiʵ��
        function s_vm_ret=vrep_matlab_link_init(obj,c_vrep,s_vm_in)
            c_vrep.simxFinish(-1); % just in case, close all opened connections

            %�򿪶˿ڲ���¼ID
            s_vm_in.dr20ID=c_vrep.simxStart('127.0.0.1',66666,true,true,5000,5);%66666��С���Ķ˿�

            %����Ƿ�򿪳ɹ�
            if (s_vm_in.dr20ID<=-1)
                fprintf('DR20ID��ȡʧ��');
                while true
                end
            end

            fprintf('link�ɹ�����0.2�룬ȷ��vrep׼������\n');
            pause(0.2);

            %��ȡ��������������ȡ λ �ˣ��������ȡ��ʵ�������״�ľ������Ϊ�״����������Ϸ���
            [ret,s_vm_in.dr20_body_handle]=c_vrep.simxGetObjectHandle(s_vm_in.dr20ID,'Hokuyo_URG_04LX_UG01',c_vrep.simx_opmode_blocking);
            if(ret~=0)
                while true
                    fprintf('dr20��������ȡʧ��\n')
                end
            end

            %��ȡ���ӵľ������������ٶȣ�
            [ret,s_vm_in.dr20_left_wheel_handle]=c_vrep.simxGetObjectHandle(s_vm_in.dr20ID,'dr20_leftWheelJoint_',c_vrep.simx_opmode_blocking);
            if (ret~=0)
                while true
                    fprintf('dr20���־����ȡʧ��\n');
                end
            end
            [ret,s_vm_in.dr20_right_wheel_handle]=c_vrep.simxGetObjectHandle(s_vm_in.dr20ID,'dr20_rightWheelJoint_',c_vrep.simx_opmode_blocking);
            if (ret~=0)
                while true
                    fprintf('dr20���־����ȡʧ��\n');
                end
            end





            % Now retrieve streaming data (i.e. in a non-blocking fashion):
            fprintf('vrep_matlab_simulation_start!\n');
            c_vrep.simxAddStatusbarMessage(s_vm_in.dr20ID,'Matlab linked!',c_vrep.simx_opmode_oneshot);%��vrep�����һ��matlab�Ѿ�����

            %��һ�ε��ñ�����streamingģʽ���ֲ���˵�ģ�
            c_vrep.simxGetObjectOrientation(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,-1,c_vrep.simx_opmode_streaming);
            c_vrep.simxGetObjectPosition(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,-1,c_vrep.simx_opmode_streaming);
            c_vrep.simxGetObjectVelocity(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,c_vrep.simx_opmode_streaming);
            [retCode,lidar_pack]=c_vrep.simxGetStringSignal(s_vm_in.dr20ID,'UG01_distance',c_vrep.simx_opmode_streaming );%for first call���ֲ���д��һ�ε���Ҫstreaming
            %��ȷ����ȷ��ȡ��һ���״��ֹ�������
            while (retCode~=c_vrep.simx_return_ok)
                [retCode,lidar_pack]=c_vrep.simxGetStringSignal(s_vm_in.dr20ID,'UG01_distance',c_vrep.simx_opmode_buffer);
            end
            s_vm_in.lidar_array_raw=c_vrep.simxUnpackFloats(lidar_pack);
            fprintf('�״��ȡ�ɹ�����ʼ�������\n')
            s_vm_ret=s_vm_in;%���½ṹ��
        end%function vrep_matlab_link_init(c_vrep)
        
        function s_vm_ret=wheel_velocity_set(obj,c_vrep,s_vm_in,which_wheel,speed)
            %���������ٶȣ�which_wheelѡ�����ӵ��ַ���'L'Ϊ���֣�'R'Ϊ���֣������ִ�Сд
            %speed Ϊ�ٶ�
            if(which_wheel=='l' || which_wheel=='L')%��
                c_vrep.simxSetJointTargetVelocity(s_vm_in.dr20ID,s_vm_in.dr20_left_wheel_handle,speed,c_vrep.simx_opmode_streaming);
            end
            if(which_wheel=='r' || which_wheel=='R')%��
                c_vrep.simxSetJointTargetVelocity(s_vm_in.dr20ID,s_vm_in.dr20_right_wheel_handle,speed,c_vrep.simx_opmode_streaming);
            end
            s_vm_ret=s_vm_in;
        end
        
        function s_vm_ret=lidar_array_update(obj,c_vrep,s_vm_in)
            [retCode,lidar_pack]=c_vrep.simxGetStringSignal(s_vm_in.dr20ID,'UG01_distance',c_vrep.simx_opmode_buffer);
            if (retCode==c_vrep.simx_return_ok)%==0 �и���
                s_vm_in.lidar_array_raw=c_vrep.simxUnpackFloats(lidar_pack);
            else%û�и���
                s_vm_in.lidar_array_raw=current_lidar_array;
            end
            s_vm_ret=s_vm_in;
        end
        
        function [s_vm_ret,distance]=lidar_point_read(obj,s_vm_in,index)
            distance=s_vm_in.lidar_array_raw(index*3-1);
            s_vm_ret=s_vm_in;
        end
        
        %���³���λ�ˣ�ʵ�������״�λ�ˣ�
        function [s_vm_ret]=car_pose_update(obj,c_vrep,s_vm_in)
            %�Ȼ�ȡz��euler
            [retCode,eulerAngles_vec3]=c_vrep.simxGetObjectOrientation(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,-1,c_vrep.simx_opmode_buffer);
            if(retCode~=0)%��ͷ�ǶȻ�ȡʧ��
                fprintf('angle_z get failed!!!\n');
            else
                s_vm_in.angle_z=eulerAngles_vec3(3);
            end

            %�ٻ�ȡ2d����
            [retCode,position3d]=c_vrep.simxGetObjectPosition(s_vm_in.dr20ID,s_vm_in.dr20_body_handle,-1,c_vrep.simx_opmode_buffer);
            if(retCode~=0)
                fprintf('car_body_pos get failed!!!\n');
            else
                s_vm_in.pos_x=position3d(1);
                s_vm_in.pos_y=position3d(2);
            end
            
            s_vm_ret=s_vm_in;

        end
        
        %���³����ٶ�
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

        %��ȡ�ϰ���
        function [s_vm_ret,yes_it_is]=is_obstacle_80cmHighWall200cm(obj,vrep,s_vm_in,pos_x,pos_y,pos_z)
            persistent is_inited;%��̬�������Ƿ��Ѿ���ʼ�����ˣ���һ�ε��õó�ʼ��һ�£���ʵ���ǵ�һ�εĶ�ȡ��ʽ��һ��
            persistent handle_200cm;
            persistent handle_200cm0;
            persistent handle_200cm1;
            persistent handle_200cm2;
            persistent handle_200cm3;
            persistent handle_200cm4;
            persistent handle_200cm5;
            persistent handle_200cm6;
            
            s_vm_ret=s_vm_in;

            RtoA=1.0/3.1415926535*180.0;%����ת�Ƕ�ϵ��
            Wall_x_length=1.5001/10.0;%x���򳤶�
            Wall_y_length=2.0000;%y���򳤶�
            Wall_z_length=8.0/10.0;%z���򳤶� ���߶�

            if isempty(is_inited)
                is_inited=false;
            end

            if(is_inited==false)%��һ�ζ�ȡ
                is_inited=true;%�´ξͲ��ǵ�һ����
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

            read_mode=vrep.simx_opmode_streaming;%������Ȼ��streaming����Ȼ��ʱ��������
            %read_mode=vrep.simx_opmode_buffer;


            retCode=-1;
            while(retCode~=0)
                [retCode,euler]=vrep.simxGetObjectOrientation(s_vm_in.dr20ID,handle_200cm,-1,read_mode);
                [retCode,wall_pos]=vrep.simxGetObjectPosition(s_vm_in.dr20ID,handle_200cm,-1,read_mode);
            end
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%��ת��90�ȵ�ǽ����ͼ�����
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%����ǽ
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
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%��ת��90�ȵ�ǽ����ͼ�����
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%����ǽ
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
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%��ת��90�ȵ�ǽ����ͼ�����
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%����ǽ
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
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%��ת��90�ȵ�ǽ����ͼ�����
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%����ǽ
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
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%��ת��90�ȵ�ǽ����ͼ�����
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%����ǽ
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
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%��ת��90�ȵ�ǽ����ͼ�����
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%����ǽ
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
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%��ת��90�ȵ�ǽ����ͼ�����
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%����ǽ
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
            if(abs(euler(3)*RtoA-90)<abs(euler(3)*RtoA-0))%��ת��90�ȵ�ǽ����ͼ�����
                x_min=wall_pos(1)-Wall_y_length/2.0;
                x_max=wall_pos(1)+Wall_y_length/2.0;
                z_min=wall_pos(3)-Wall_z_length/2.0;
                z_max=wall_pos(3)+Wall_z_length/2.0;
                y_min=wall_pos(2)-Wall_x_length/2.0;
                y_max=wall_pos(2)+Wall_x_length/2.0;
            else%����ǽ
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

            yes_it_is=false;%�����κε�ǽ���У������ǿ�����������С���Լ���
            s_vm_ret=s_vm_in;
        end
        
        %�����״���λ����Ϣ��̬��ͼ
        function [s_vm_ret]=dynamic_mapping(obj,s_vm_in)
            AtoR=1.0/180.0*3.1415926535;
            dilate_neighborhood_mode=0;%����ģʽ��0�����ͣ�1������2������
            pixel_obstacle_value=127;%�����⵽�ϰ����������������ֵ
            for i=1:684
                [s_vm_in,distance]=obj.lidar_point_read(s_vm_in,i);%��ȡ�����ľ���
                if(distance>0)%������������Ч��������������0��
                    absolute_angle=AtoR*((i-1)*240/(684-1)+(-120))+s_vm_in.angle_z;%�������״��ľ��ԽǶȣ����ȣ����복�峯���޹أ�
                    lidar_pose_x=s_vm_in.pos_x;
                    lidar_pose_y=s_vm_in.pos_y;
                    obstacle_x=lidar_pose_x+distance*cos(absolute_angle-0*AtoR);
                    obstacle_y=lidar_pose_y+distance*cos(absolute_angle-90*AtoR);
                    pixel_x=round(obstacle_x*100.0);%���ؾ���1cm����������ȡ��
                    pixel_y=round(obstacle_y*100.0);
                    if dilate_neighborhood_mode==0%������
                        if(pixel_x>0 && pixel_x<=500 && pixel_y>0 &&pixel_y<=500)
                            s_vm_in.map(500-pixel_y,pixel_x)=pixel_obstacle_value;
                        end
                    elseif dilate_neighborhood_mode==1%����������
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
                        
                    elseif dilate_neighborhood_mode==2%����������
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

