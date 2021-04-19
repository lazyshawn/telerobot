function TeleRobot
close all
addpath('./spacedyn_v1r2');   %the path of  SpaceDyn
stlfilePath='./stl';          %the path holding the stl files of the robots

global Qi J_type BB SS SE S0
global cc c0 ce
global m0 inertia0 m inertia mass
global d_time
global Qe
global Gravity
global num_q Ez

%   z轴上的单位向量
Ez = [ 0 0 1 ]';

%%% 定义连杆参数
%   各连杆连通性
BB = [ 0 1 2];
%   定义父子link
SS = [ -1 1 0;
       0 -1 1
       0 0 -1;];
%   定义连接基座的的连杆为link0,连接执行器的连杆为link2
S0 = [ 1 0 0];
SE = [ 0 0 1];
%   定义关节为旋转关节(R为旋转副P为移动副)
J_type = [ 'R' 'R' 'R'];
%   基座坐标系到link0坐标系的变换(x正向移动0.5m),未连接到基座的关节值为0 0 0
c0 = [ 0.5 0 0; 
       0  0  0; 
       0  0  0];
%   定义发布者和消息
positon1 = rospublisher('weixing/joint1_position_controller/command', 'std_msgs/Float64');
positon2 = rospublisher('weixing/joint2_position_controller/command', 'std_msgs/Float64');
positon3 = rospublisher('weixing/joint3_position_controller/command', 'std_msgs/Float64');
msg1 = rosmessage(positon1);
msg2 = rosmessage(positon2);
msg3 = rosmessage(positon3);   
%   卫星质量(kg)和转动惯量i=m*s*s/6
m0 = 500; 
inertia0 = [ 41.67 0 0;
             0 41.67 0;
             0 0 41.67]*1;
%   各连杆的质量         
m = [ 50 25 25 ];
%   总质量
mass = sum(m) + m0;

Qi = [ 0 0 0;
       0 0 0;
       0 0 0]; % relationship 
%   各连杆转动惯量
inertia = [ 0.0625 0 0 0.0625 0 0 0.0625 0 0;
            0 4.1979 0 0 4.1979 0 0 4.1979 0;
            0 0 4.1979 0 0 4.1979 0 0 4.1979];
%%% 定义各连杆长度,生成一个3*3*3的零矩阵
cc = zeros(3,3,3);

cc(:,1,1) = [ -0.5 0 0 ]';
cc(:,1,2) = [ 0.5 0 0 ]';
cc(:,2,2) = [ -0.5 0 0 ]';
cc(:,2,3) = [0.5 0 0]';
cc(:,3,3) = [-0.5 0 0]';
ce = [ 0 0 0.5;
       0 0 0;
       0 0 0];

Qe = [ 0 0 0;
       0 0 0;
       0 0 0];
   
%%% 初始化变量
q = [-3*pi/16 3*pi/16 -3*pi/16]';
qd = zeros(3,1);
%qdd = zeros(2,1);
%   初始化基座速度,角速度
v0 = [ 0 0 0 ]';
w0 = [ 0 0 0 ]';
vd0 = [ 0 0 0 ]';
wd0 = [ 0 0 0 ]';
%   初始化连杆速度,角速度
vv = zeros(3,3);
ww = zeros(3,3);
vd = zeros(3,3);
wd = zeros(3,3);

R0 = [ 0 0 0 ]';
Q0 = [ 0 0 0 ]';
A0 = eye(3,3);
Te = zeros(3,3);
Fe = zeros(3,3);
F0 = [ 0 0 0 ]';
T0 = [ 0 0 0 ]';
tau = [ 0 0 0]';
%   重力
Gravity = [ 0 0 0 ]';

%%% 控制关节的参数
Kp = 500;
Kd = 120;

%%% 连杆数量
num_q = length( q );
num_e = 1;
joints = j_num(num_e);

%%% 打开文件
fidw = fopen('sample.dat','w');

%%%%%%%%%%%%%%%
%%% 仿真开始 %%%
%%%%%%%%%%%%%%%

% 采样一步的时间(s)
d_time = 0.01;
samplingT = d_time;

fig=figure;
% 初始化动画渲染
[faces, vertices, colors, p0, p1,p2, p3] = animation_ini(A0, R0, q);
 view(-90,90);
 pause();
 str1='Time:';
 str2=num2str(0*samplingT,'%.01f');
 str3=strcat(str1,str2);
 str4=strcat(str3,'[s]');
 time_text=text(-1000.0,2900.0,1000.0,str4,'FontName','Times New Roman','fontsize',20,'EraseMode','normal');

 counter=0;
 frame_counter=1;
 desired_qi= [-3*pi/16 3*pi/16 -3*pi/16];
 %  延时
 time_delay=10; % unit s
 %  命令缓冲区,可存储10条命令,先进先出FIFO,[角度1,角度2,角度3,计数器]
 command_buff=zeros(10,4);
 %  缓存区结束
 command_buff_end=0;
 %  延时 = DELAY_20*0.01s(d_time?)
 DELAY_T=1;
 for i=1:10
     command_buff(i,4)=DELAY_T;     
 end


%   设置键盘回调和移动标志
set(fig,'WindowKeyPressFcn',@KeyPress,'WindowKeyReleaseFcn', @KeyRelease);
         fig.UserData.q = false;
         fig.UserData.a = false;
         fig.UserData.w = false;
         fig.UserData.s = false;
         fig.UserData.e = false;
         fig.UserData.d = false;
         
         fig.UserData.r = false;
         fig.UserData.f = false;
         fig.UserData.t = false;
         fig.UserData.g = false;
         
         previous_key_counter=0;
         key_counter_first=0; 
         desired_qi_buff=desired_qi;
%%%%%%%%%%%%%%%         
%%% 循环开始 %%%
%%%%%%%%%%%%%%%
while(ishandle(fig))

 counter=counter+1; 
%   检测QAWSED输入
   input_flag=0;
   if fig.UserData.q
      desired_qi_buff(1)=desired_qi_buff(1)+1*pi/180;  % increase joint angle 1 by 1
      input_flag=1;
   end
   if fig.UserData.a
      desired_qi_buff(1)=desired_qi_buff(1)-1*pi/180;  % decrease joint angle 1 by 1
      input_flag=1;
   end
   if fig.UserData.w
      desired_qi_buff(2)=desired_qi_buff(2)+1*pi/180;   % increase joint angle2 by 1
      input_flag=1;
   end
   if fig.UserData.s
      desired_qi_buff(2)=desired_qi_buff(2)-1*pi/180;  % decrease joint angle 2 by 1
      input_flag=1;
   end
   if fig.UserData.e
      desired_qi_buff(3)=desired_qi_buff(3)+1*pi/180;  % increase joint angle 3 by 1
      input_flag=1;
   end
   if fig.UserData.d
      desired_qi_buff(3)=desired_qi_buff(3)-1*pi/180;  % decrease joint angle 3 by 1
      %fprintf('d');
      input_flag=1;
   end
   
   % 检测RFTG末端控制
   if fig.UserData.r
      Jacobian = calc_gj( R0, RR, A0, AA, q, num_e );
      Jacobian = Jacobian(1:3,:);
      QD=pinv(Jacobian)*[20 0 0]';
      dc=d_time.*QD;
      desired_qi_buff(1)=desired_qi_buff(1)+dc(1)*pi/180;
      desired_qi_buff(2)=desired_qi_buff(2)+dc(2)*pi/180;
      desired_qi_buff(3)=desired_qi_buff(3)+dc(3)*pi/180;
      input_flag=1;
   end
   if fig.UserData.f      
      Jacobian = calc_gj( R0, RR, A0, AA, q, num_e );
      Jacobian = Jacobian(1:3,:);
      QD=pinv(Jacobian)*[-20 0 0]';
      dc=d_time.*QD;
      desired_qi_buff(1)=desired_qi_buff(1)+dc(1)*pi/180;
      desired_qi_buff(2)=desired_qi_buff(2)+dc(2)*pi/180;
      desired_qi_buff(3)=desired_qi_buff(3)+dc(3)*pi/180;
      input_flag=1;
   end
   if fig.UserData.t
      Jacobian = calc_gj( R0, RR, A0, AA, q, num_e );
      Jacobian = Jacobian(1:3,:);
      QD=pinv(Jacobian)*[0 20 0]';
      dc=d_time.*QD;
      desired_qi_buff(1)=desired_qi_buff(1)+dc(1)*pi/180;
      desired_qi_buff(2)=desired_qi_buff(2)+dc(2)*pi/180;
      desired_qi_buff(3)=desired_qi_buff(3)+dc(3)*pi/180;
      input_flag=1;
   end
   if fig.UserData.g
      Jacobian = calc_gj( R0, RR, A0, AA, q, num_e );
      Jacobian = Jacobian(1:3,:);
      QD=pinv(Jacobian)*[0 -20 0]';
      dc=d_time.*QD;
      desired_qi_buff(1)=desired_qi_buff(1)+dc(1)*pi/180;
      desired_qi_buff(2)=desired_qi_buff(2)+dc(2)*pi/180;
      desired_qi_buff(3)=desired_qi_buff(3)+dc(3)*pi/180;
      input_flag=1;
   end
   
   if(command_buff_end>0)
      if(command_buff(1,4)==0) % It needs to push the command outof the buffer
         desired_qi=[command_buff(1,1),command_buff(1,2),command_buff(1,3)];
        % test code
        %   for i=1:command_buff_end
        %    fprintf('counter:%d out:%f %f %f %d \n',counter,command_buff(i,:));
        %   end
        % test code 
            command_buff_end=command_buff_end-1;
          for i=1:command_buff_end
           command_buff(i,:)=command_buff(i+1,:);  
          end
      end
   end
   
    
   % in test, it is founded that one press of key will triger several
   % callbacks. The following part is used to solve the problem 
   if(input_flag==1)
     if(key_counter_first==0)
     previous_key_counter=counter;
     key_counter_first=1;
     else
        if(counter-previous_key_counter<50)
         input_flag=0;
        end    
     end    
   end
   
   
   if(input_flag==1) % command is updated
       command_buff_end=command_buff_end+1;
       command_buff(command_buff_end,:)=[desired_qi_buff(1),desired_qi_buff(2),desired_qi_buff(3),DELAY_T];
       previous_key_counter=counter;
       % test code
       % for i=1:command_buff_end
       % fprintf('counter:%d in:%f %f %f %d \n',counter,command_buff(i,:));
       % end
       % test code 
   end
   
   for i=1:command_buff_end
       command_buff(i,4)=command_buff(i,4)-1; % in each loop the counter corresponding to each command will 
   end                                        % be decreased by 1, when it becomes to be 0, the corresponding 
                                              % will be pushed out

                                              
                                              
    
   tau = Kp.*( desired_qi' - q ) - Kd.*qd;  % a PD controller for each of the joint

   %%% Forward dynamics
   [ R0, A0, v0, w0, q, qd ] = f_dyn_rk2( R0, A0, v0, w0, q, qd, F0, T0, Fe, Te, tau );

    AA = calc_aa( A0, q );
    RR = calc_pos( R0, A0, AA, q );
    [ POS_e, ORI_e ] = f_kin_e( RR, AA, joints );

    POS_e_Old = POS_e;
    %%% Euler angle
    Q0 = dc2rpy( A0' );
    %发送消息
    msg1.Data=q(1);
    msg2.Data=-q(2);
    msg3.Data=q(3);
    send(positon1,msg1);
    send(positon2,msg2);
    send(positon3,msg3);
    % animation frame
    Q0 = dc2rpy(A0');
    Z0 = dc2rpy((ORI_e)');
    
    q_deg = q * 180/pi;
    qd_deg = qd * 180/pi;
    Q0_deg = Q0 * 180/pi;
    Z0_deg = Z0 * 180/pi;
    desired_qi_deg = desired_qi * 180/pi;
    time=counter*0.01;
    % for save the necessary state in file. 
    %fprintf(fidw,'%g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g\n',time,Q0_deg,R0,POS_e,q_deg,Z0_deg,tau,qd_deg,desired_qi_deg);
    robot_plot( faces, vertices, colors, A0, AA, R0, q_deg, p0, p1,p2,p3);
    str1='Time:';
    str2=num2str(counter*samplingT,'%.01f');
    str3=strcat(str1,str2);
    str4=strcat(str3,'[s]');
    set(time_text,'String',str4);
    if(time==10)
     break;
    end
    
 pause(0.01);
 end
     
fclose(fidw);
tmp=dlmread('sample.dat');

%%% Plot graphs
figure,
plot(tmp(:,1),tmp(:,2:4),'-');
title('Q0');
xlabel('Time [s]');
ylabel('Rotation of Base [deg]');
legend('Around x axis','Around y axis','Around z axis');

figure,
plot(tmp(:,1),tmp(:,5:7),'-');
title('R0');
xlabel('Time [s]');
ylabel('Position of Base [m]');
legend('x','y','z');

figure,
plot(tmp(:,1),tmp(:,8:10),'-');
title('POS_e');
xlabel('Time [s]');
ylabel('Position of End-effecter [m]');
legend('x','y','z');

figure,
plot(tmp(:,9),tmp(:,8),'-r','LineWidth',3);
title('End effector trajectory');
axis equal;
set(gca,'XLim',[-1.5 1.5]);
set(gca,'YLim',[-1 4]);
set(gca,'XDir','reverse')
xlabel('Y');
ylabel('X');
%legend('x','y','z');

figure,
plot(tmp(:,1),tmp(:,11:13),'-');
title('q');
xlabel('Time [s]');
ylabel('Angle of joints [deg]');
legend('Joint1','Joint2','Joint3');


% figure,
% plot(tmp(:,1),tmp(:,14:16),'-');
% title('Z0');
% xlabel('Time [s]');
% ylabel('Endpoint orientation [deg]');
% legend('roll','pitch','yaw'); 

% figure,
% plot(tmp(:,1),tmp(:,17:19),'-');
% title('torque');
% xlabel('Time [s]');
% ylabel('Torque [Nm]');
% legend('joint1','joint2','joint3'); 

figure,
plot(tmp(:,1),tmp(:,20:22),'-');
title('angle velocity of joints');
xlabel('Time [s]');
ylabel('angle velocity of joints[m/s]');
legend('joint1','joint2','joint3'); 

% figure,
% plot(tmp(:,1),tmp(:,23:25),'-');
% title('angle acceleration of joints');
% xlabel('Time [s]');
% ylabel('angle acceleration of joints[m/s]');
% legend('joint1','joint2','joint3'); 

end


% keyboard callback
function KeyPress(varargin)
     fig = varargin{1};
     key = varargin{2}.Key;
     if strcmp(key,'q') 
         fig.UserData.q = true;
     elseif strcmp(key,'a')
         fig.UserData.a = true;
     elseif strcmp(key,'w')
         fig.UserData.w = true;
     elseif strcmp(key,'s')
         fig.UserData.s = true;
     elseif strcmp(key,'e')
         fig.UserData.e = true;
     elseif strcmp(key,'d')
         fig.UserData.d = true;
     elseif strcmp(key,'r')
         fig.UserData.r = true;
     elseif strcmp(key,'f')
         fig.UserData.f = true;
     elseif strcmp(key,'t')
         fig.UserData.t = true;
     elseif strcmp(key,'g')
         fig.UserData.g = true;
     end
end

% keyboard callback
function KeyRelease(varargin)
     fig = varargin{1};
     key = varargin{2}.Key;
     if strcmp(key,'q') 
         fig.UserData.q = false;
     elseif strcmp(key,'a')
         fig.UserData.a = false;
     elseif strcmp(key,'w')
         fig.UserData.w = false;
     elseif strcmp(key,'s')
         fig.UserData.s = false;
     elseif strcmp(key,'e')
         fig.UserData.e = false;
     elseif strcmp(key,'d')
         fig.UserData.d = false;
     elseif strcmp(key,'r')
         fig.UserData.r = false;
     elseif strcmp(key,'f')
         fig.UserData.f = false;
     elseif strcmp(key,'t')
         fig.UserData.t = false;
     elseif strcmp(key,'g')
         fig.UserData.g = false;
     end
end
