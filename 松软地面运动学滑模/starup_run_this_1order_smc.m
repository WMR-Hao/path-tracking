%  主函数 star up
%  2020 11 29 
clear all
close all
clc 

%% 初始化
ts=0.01;    Tsim=20;   t=0:ts:Tsim;
X=0.4 ;   Y=0;    ALPHA=pi/2;  % 初始位置
vx=0; dot_alpha=0;vy=0;

wheel_r = 0.11;
omega_max=6.4; % 6.4
%% 参考轨迹部分
ref_position=zeros(length(t),3);
ref_position(1,:)=[0 0 pi/4];
vxr=0.5;
vyr=0;
% omegar=0.5;
omegar=0.5;

for i=1:length(t)
    [Xr,Yr,ALPHAr] = ref_path(vxr,omegar,ts,ref_position(i,1),ref_position(i,2),ref_position(i,3));
    ref_position(i+1,1)=Xr;
    ref_position(i+1,2)=Yr;
    ref_position(i+1,3)=ALPHAr;
end

figure(1)   % 位置图
plot(ref_position(:,1),ref_position(:,2),'r-');
hold on

%% Sampling time

% X(0)=0;
% Y(0)=0;
% ALPHA(0)=0;

for i=1:length(t)
    %% 读传感器值
    body_pos(i,1)=X;
    body_pos(i,2)=Y;
    body_pos(i,3)=ALPHA;
    body_vel(i,1)=vx;
    body_vel(i,2)=vy;   
    body_vel(i,3)=dot_alpha;
    
    Xr=ref_position(i,1);
    Yr=ref_position(i,2);
    ALPHAr= ref_position(i,3);
    vxr=vxr;
    vyr=vyr;
    dot_alphar=omegar;
    
    if i<2500&& i>2000
    vy=0.2;
    end
    [vx,dot_alpha]=the_1_order_smc_controller(vxr,dot_alphar,ALPHAr,ALPHA,X,Xr,Y,Yr,vy);
    
   [X,Y,ALPHA,vx,dot_alpha]=kinematics_model_slide...
       (X,Y,ALPHA,vx,vy,dot_alpha,ts);

    if i>2
        plot([body_pos(i-1,1) body_pos(i,1)],[body_pos(i-1,2) body_pos(i,2)],'black.-');
        hold on
    end
    drawnow
    
end

%  %% 对结果作图
% figure(2)   % 位置图
% subplot(2,2,1)
% plot(body_position(:,1),body_position(:,2),'r-',ref(:,1),ref(:,2),'b-');
% xlabel('X轴');
% ylabel('Y轴');
% title('机器人路径 ');
% legend('实际位置','参考位置');
% hold on
% 
% subplot(2,2,2)   %车速
% plot(tout,body_vel(:,1),'r-',tout,ref_vel(:,1),'b-');
% xlabel('时间');
% ylabel('车速');
% title('机器人车速变化 ');
% legend('实际车速','参考车速');
% hold on
% 
% subplot(2,2,3)   %转动角速度
% plot(tout,body_vel(:,3),'r-',tout,ref_vel(:,3),'b-');
% xlabel('时间');
% ylabel('车速');
% title('机器人转向速度变化 ');
% legend('实际转速','参考转速');
% hold on
% 
% subplot(2,2,4)   %误差
figure(10)
e_x=body_pos(:,1)-ref_position(1:end-1,1);
e_y=body_pos(:,2)-ref_position(1:end-1,2);
e_alpha=body_pos(:,3)-ref_position(1:end-1,3);
plot(t,e_x,'r-',t,e_y,'b',t,e_alpha,'g',t,zeros(1,size(t,1)),'--');
xlabel('时间');
ylabel('误差');
title('机器人误差变化 ');
legend('x方向误差','y方向误差','转角误差');

% figure(2)
% subplot(2,2,1) 
% e_v=body_vel(:,1)-ref_vel(:,1);
% e_omega=body_vel(:,3)-ref_vel(:,3);
% plot(tout,e_v,'r',tout,e_omega,'b');
% xlabel('时间');
% ylabel('误差');
% title('机器人速度误差变化 ');
% legend('速度误差','转动误差');
% 
% subplot(2,2,2) 
% plot(tout,control_u(:,1),'r',tout,control_u(:,1),'b');
% xlabel('时间');
% ylabel('轮速');
% title('机器人左侧轮速变化 ');
% legend('实际转速','参考转速');
% 
% subplot(2,2,3)
% plot(tout,control_u(:,2),'r',tout,control_u(:,2),'b');
% xlabel('时间');
% ylabel('轮速');
% title('机器人右侧轮速变化 ');
% legend('实际转速','参考转速');

% %%
% figure(1)   % 位置图
% plot(body_position(:,1),body_position(:,2),'r-',ref(:,1),ref(:,2),'b-');
% xlabel('X轴');
% ylabel('Y轴');
% title('机器人路径 ');
% legend('实际位置','参考位置');
% hold on
% 
% figure(2)    %车速
% plot(tout,body_vel(:,1),'r-',tout,ref_vel(:,1),'b-');
% xlabel('时间');
% ylabel('车速');
% title('机器人车速变化 ');
% legend('实际车速','参考车速');
% hold on
% 
% figure(3)    %转动角速度
% plot(tout,body_vel(:,3),'r-',tout,ref_vel(:,3),'b-');
% xlabel('时间');
% ylabel('车速');
% title('机器人转向速度变化 ');
% legend('实际转速','参考转速');
% hold on