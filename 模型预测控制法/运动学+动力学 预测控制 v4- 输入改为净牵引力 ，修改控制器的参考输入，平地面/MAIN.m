%  主函数框架 star up
%  2020 11 29 
clc
close all
clear all

%% 初始化
dt = 0.032 ;    T = 32*1.5*1*0.7 ;    t = 0:dt:T;% T = 32*1*0.7 ;
X = 1 ;   Y = 0;    PSI = pi/2;  % 初始位置    [1 0 pi/4];
vx = 0.1;  vy = 0;    gamma=0;
alpha=deg2rad(0);

wheel_r = 0.11;
gamma_max = 15;  % 6.4

Fdpl=0;  % 净牵引力
Fdpr=0;

% vx1=(0.5*ones(size(t,2),1))';
% gamma1=sin(t);

%  Faa=1*sin(t);
% % Taa=(6.5*ones(size(t,2),1))';% 6.4746
%  Fbb=(0*ones(size(t,2),1))';

%% 创建记录数据的输出文件  
% 标题 需要先创建simdata文件夹
ERROR = fopen('simdata\error.txt','w');
fprintf(ERROR, "time  , ex  , ey  , ealpha  , evx ,  evy , eomega  \n");
fclose(ERROR);

Pos_Vel = fopen('simdata\pos_vel.txt','w');
fprintf(Pos_Vel, "time  , ref_x  , ref_y  , ref_alpha  , x  , y  , alpha  , ref_vx  , ref_vy  , ref_omega  , vx  , vy  , omega  \n");
fclose(Pos_Vel);

control_U = fopen('simdata\controller_u.txt','w');
fprintf(control_U, "time  , u_vx  , u_omega  \n");
fclose(control_U);

%% 参考轨迹部分
tref = 0:dt:T+20;
ref_position=zeros(length(tref),3);
ref_position(1,:)=[0 0 pi/4];
vxr=0.5;
vyr=0;
gammar=0.2;
ref_velocity=zeros(length(tref),3);
ref_velocity(1,:)=[vxr vyr gammar];

for i=1:length(tref)
    [Xr,Yr,ALPHAr] = ref_path(vxr,gammar,dt,ref_position(i,1),ref_position(i,2),ref_position(i,3));
    ref_position(i+1,1)=Xr;
    ref_position(i+1,2)=Yr;
    ref_position(i+1,3)=ALPHAr;
    ref_velocity(i+1,1)=vxr;
    ref_velocity(i+1,2)=vyr;
    ref_velocity(i+1,3)=gammar;
end

figure(10)   % 位置图
plot(ref_position(:,1),ref_position(:,2),'m-');
hold on

%% 存储位姿数据和速度数据
body_pos=zeros(length(t),3);
body_vel=zeros(length(t),3);

error_pos = zeros(length(t),3);
error_vel = zeros(length(t),3);

BETA_vector=zeros(length(t),5);

%% 控制
% u=zeros(length(t),4);% u=[vx omega omega_l omega_r]

for i=1:length(t)
    %% 读传感器值
    body_pos(i,1)= X ;    % body_pos=[X Y PSI]
    body_pos(i,2)= Y ;
    body_pos(i,3)= PSI ;
    body_vel(i,1)= vx ;   %body_vel=[vx vy gamma]
    body_vel(i,2)= vy ;
    body_vel(i,3)= gamma ;
    
    %% 控制器 计算
    %tic
%     %************************   控制器部分  *************************
        u_out = my_dyn_MPC_controller...
          (body_pos(i,:),body_vel(i,:),ref_position(i+1:end,:),ref_velocity(i+1:end,:),Fdpl,Fdpr,dt,alpha)
 
    Fdpl=u_out(1);
    Fdpr=u_out(2);
 
%% 执行器赋值    模拟机器人运动
    [X,Y,PSI,vx,vy,gamma,beta]=dynamics_model(Fdpl,Fdpr,X,Y,PSI,vx,vy,gamma,dt,alpha);

%%  计算误差
error_pos(i,1) = X - ref_position(i+1,1);
error_pos(i,2) = Y - ref_position(i+1,2);
error_pos(i,3) = PSI - ref_position(i+1,3);

error_vel(i,1) = vx - vxr;
error_vel(i,2) = vy - vyr;
error_vel(i,3) = gamma - gammar;

% 记录车轮侧偏角
BETA_vector(i,:)=beta;
%% 存储数据至simdata文件
% ERROR = fopen('error.txt','a');
% fprintf(ERROR,'%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',error(i,:));
% fclose(ERROR);  

% Pos_Vel = fopen('pos_vel.txt','a');
% fprintf(Pos_Vel,'%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',pos_vel(i,:));
% fclose(Pos_Vel);  

% control_U = fopen('controller_u.txt','a');
% fprintf(control_U,'%.6f  ,  %.6f,  %.6f  \n',controller_u(i,:));
% fclose(control_U);

%% 



%     % 作出轨迹图像
    if i>2
        figure(10)
        plot([body_pos(i-1,1) body_pos(i,1)],[body_pos(i-1,2) body_pos(i,2)],'b.-'); %'black.-'  'red.-'
        xlabel('X轴 m');
        ylabel('Y轴 m');
        title('轨迹图');
        axis equal
        hold on
        
% %         figure(11)
%         plot([t(i-1) t(i)],[body_vel(i-1,1) body_vel(i,1)],'black.-',...
%              [t(i-1) t(i)],[body_vel(i-1,2) body_vel(i,2)],'red.-',...
%              [t(i-1) t(i)],[body_vel(i-1,3) body_vel(i,3)],'green.-'); %'black.-'  'red.-'
%         xlabel('X轴 m');
%         ylabel('Y轴 m');
%         title('速度图');
%         legend('纵向速度','横向速度','转动角速度');
%         hold on
% 
% %         figure(2) %位姿误差 
%         plot( [t(i-1) t(i)], [error_pos(i-1,1) error_pos(i,1)],'blue.-',...
%               [t(i-1) t(i)], [error_pos(i-1,2) error_pos(i,2)],'red.-',...
%               [t(i-1) t(i)], [error_pos(i-1,3) error_pos(i,3)],'green.-');
%         hold on
%         
%         figure(3) % 速度误差
%         plot( [t(i-1) t(i)], [error_vel(i-1,1) error_vel(i,1)],'blue.-',...
%               [t(i-1) t(i)], [error_vel(i-1,2) error_vel(i,2)],'red.-',...
%               [t(i-1) t(i)], [error_vel(i-1,3) error_vel(i,3)],'green.-');
%         hold on
%  
    end
    
  
    drawnow 


% % %%  
    
    i
end

%% 分析结果用：
% %% 画图

        figure(12) %位姿误差 
        plot( t, error_pos(:,1),'blue.-',...
              t, error_pos(:,2),'red.-',...
              t, error_pos(:,3),'green.-');
        hold on
        xlabel('时间 t');
        ylabel('位姿 ');
        title('位姿误差图');
        legend('X方向误差','Y方向误差','转动角度误差');
       
        
        figure(13) % 速度误差
        plot( t, error_vel(:,1),'blue.-',...
              t, error_vel(:,2),'red.-',...
              t, error_vel(:,3),'green.-');
        hold on
        xlabel('时间 t');
        ylabel('速度  ');
        title('速度误差图');
        legend('纵向速度误差','横向速度误差','转动角速度误差');

figure(14)   % 位置图
% subplot(2,2,1)
plot(body_pos(:,1),body_pos(:,2),'.-',ref_position(:,1),ref_position(:,2),'r-');
xlabel('X轴');
ylabel('Y轴');
title('机器人路径 '); 
legend('实际位置','参考位置');
hold on


figure(15)   % 速度图
subplot(2,1,1)
plot(t,body_vel(:,1),'.-',tref,ref_velocity(2:end,1),'r-');
xlabel('X轴');
ylabel('Y轴');
title('机器人速度 '); 
legend('实际前进速度','参考前进速度');
hold on

subplot(2,1,2)
plot(t,body_vel(:,3),'.-',tref,ref_velocity(2:end,3),'r-');
xlabel('X轴');
ylabel('Y轴');
title('机器人速度 '); 
legend('实际转向角速度','参考转向角速度');
hold on

figure(16)
plot(t,BETA_vector(:,1),'r--',t,BETA_vector(:,2),'b',t,BETA_vector(:,3),'black',t,BETA_vector(:,4),'g',t,BETA_vector(:,5),'m');
xlabel('X轴');
ylabel('Y轴');
title('机器人beta角 '); 
legend('质心','1轮','2轮','3轮','4轮');
hold on;


