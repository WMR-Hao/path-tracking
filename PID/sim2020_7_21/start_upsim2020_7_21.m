% 2020 7 21
% 基于动力学模型的模型预测控制 
% 机器人参数见文件夹中pdf中的文件
% 

clear all
% close all
clc 

%% 运行仿真
 sim('sim2020_7_21.slx'); % 动力学模型 mpc

%%  对结果作图
figure(1)   % 位置图
subplot(2,2,1)
plot(body_position(:,1),body_position(:,2),'r-',ref_q(:,1),ref_q(:,2),'b-');
xlabel('X轴');
ylabel('Y轴');
title('机器人轨迹');
legend('实际','参考');
hold on

subplot(2,2,2) %x方向
plot(tout,body_position(:,1),'r-',tout,ref_q(:,2),'b-');
xlabel('时间');
ylabel('变化量');
title('机器人x坐标 ');
legend('实际坐标','参考坐标');
hold on

subplot(2,2,3)   %y方向
plot(tout,body_position(:,2),'r-',tout,ref_q(:,2),'b-');
xlabel('时间');
ylabel('y轴坐标');
title('机器人y轴方向坐标变化 ');
legend('实际坐标','参考坐标');
hold on

subplot(2,2,4)   %横摆角
plot(tout,body_position(:,3),'r-',tout,ref_q(:,3),'b-');
xlabel('时间');
ylabel('偏航角');
title('机器人偏航角变化 ');
legend('实际角度','参考角度');

figure(2)
subplot(2,2,1) 
e_vx=body_vel(:,1)-ref_vel(:,1);
e_vy=body_vel(:,2)-ref_vel(:,2);
e_omega=body_vel(:,3)-ref_vel(:,3);
plot(tout,e_vx,'r',tout,e_vy,'g',tout,e_omega,'b');
xlabel('时间');
ylabel('误差');
title('机器人速度误差变化 ');
legend('纵向速度误差','横向速度误差','转动误差');

subplot(2,2,2)
plot(tout,body_vel(:,1),'r',tout,ref_vel(:,1),'b');
xlabel('时间');
ylabel('车速');
title('机器人纵向速度变化 ');
legend('实际纵向速度','参考纵向速度');

subplot(2,2,3)
plot(tout,body_vel(:,2),'r',tout,ref_vel(:,2),'b');
xlabel('时间');
ylabel('车速');
title('机器人横向速度变化 ');
legend('实际横向速度','参考横向速度');

subplot(2,2,4)
plot(tout,control_u(:,2),'r',tout,ref_vel(:,3),'b');
xlabel('时间');
ylabel('转向速度');
title('机器人转向角速度变化 ');
legend('实际转向角速度','参考转向角速度');

figure(3)
subplot(2,2,1)
e_pos=body_position-ref_q;
plot(tout,e_pos(:,1),'r',tout,e_pos(:,2),'b',tout,e_pos(:,3),'g')
xlabel('时间');
ylabel('误差值');
title('机器人位姿误差变化 ');
legend('x轴','y轴','alpha角');

% 对结果作图（动力学）
subplot(2,2,2) 
plot(tout,control_u(:,1),'r',tout,ref_f(:,1),'b');
xlabel('时间');
ylabel('轮速');
title('机器人左侧牵引力变化 ');
legend('实际牵引力','参考牵引力');

subplot(2,2,3)
plot(tout,control_u(:,2),'r',tout,ref_f(:,2),'b');
xlabel('时间');
ylabel('轮速');
title('机器人右侧牵引力变化 ');
legend('实际牵引力','参考牵引力');


% %% 
% error=[tout e_pos e_vx e_omega];
% pos_vel=[tout ref_q body_position ref_vel body_vel];
% controller_u=[tout control_u];
% 
% %% 输出文件 
% % 标题
% ERROR = fopen('error.txt','w');
% fprintf(ERROR, "time  , ex  , ey  , ealpha  , evx  , eomega  \n");
% fclose(ERROR);
% 
% Pos_Vel = fopen('pos_vel.txt','w');
% fprintf(Pos_Vel, "time  , ref_x  , ref_y  , ref_alpha  , x  , y  , alpha  , ref_vx  , ref_vy  , ref_omega  , vx  , vy  , omega  \n");
% fclose(Pos_Vel);
% 
% control_U = fopen('controller_u.txt','w');
% fprintf(control_U, "time  , u_vx  , u_omega  \n");
% fclose(control_U);
% 
% %  写入数据
% for i=1:size(tout,1)
% ERROR = fopen('error.txt','a');
% fprintf(ERROR,'%.6f  ,  %.6f  ,  %.6f  ,  %.6f  ,  %.6f  ,  %.6f  \n',error(i,:));
% fclose(ERROR);  
% 
% Pos_Vel = fopen('pos_vel.txt','a');
% fprintf(Pos_Vel,'%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',pos_vel(i,:));
% fclose(Pos_Vel);  
% 
% control_U = fopen('controller_u.txt','a');
% fprintf(control_U,'%.6f  ,  %.6f,  %.6f  \n',controller_u(i,:));
% fclose(control_U); 
% end
% 
% % 
% %%  对结果作图（动力学MPC）
% 
% figure(1)   % 位置图
% subplot(2,2,1)
% plot(body_position(:,1),body_position(:,2),'r-',ref_q(:,1),ref_q(:,2),'b-');
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
% e_x=body_position(:,1)-ref_q(:,1);
% e_y=body_position(:,2)-ref_q(:,2);
% e_alpha=body_position(:,3)-ref_q(:,3);
% plot(tout,e_x,'r-',tout,e_pos(),'b',tout,e_alpha,'g',tout,zeros(1,size(tout,1)),'--');
% xlabel('时间');
% ylabel('误差');
% title('机器人误差变化 ');
% legend('x方向误差','y方向误差','转角误差');
% 
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
% plot(tout,control_u(:,1),'r',tout,ref_f(:,1),'b');
% xlabel('时间');
% ylabel('轮速');
% title('机器人左侧牵引力变化 ');
% legend('实际牵引力','参考牵引力');
% 
% subplot(2,2,3)
% plot(tout,control_u(:,2),'r',tout,ref_f(:,2),'b');
% xlabel('时间');
% ylabel('轮速');
% title('机器人右侧牵引力变化 ');
% legend('实际牵引力','参考牵引力');
% 
% subplot(2,2,4)
% plot(tout,e_pos(:,1),'r',tout,e_pos(:,2),'b',tout,e_pos(:,3),'g')
% xlabel('时间');
% ylabel('误差值');
% title('机器人位姿误差变化 ');
% legend('x轴','y轴','alpha角');
% 


%% 
error=[tout e_pos e_vx e_vy e_omega];
pos_vel=[tout ref_q body_position ref_vel body_vel];
controller_u=[tout control_u];

%% 输出文件 
% 标题
ERROR = fopen('error.txt','w');
fprintf(ERROR, "time  , ex  , ey  , ealpha  , evx ,  evy , eomega  \n");
fclose(ERROR);

Pos_Vel = fopen('pos_vel.txt','w');
fprintf(Pos_Vel, "time  , ref_x  , ref_y  , ref_alpha  , x  , y  , alpha  , ref_vx  , ref_vy  , ref_omega  , vx  , vy  , omega  \n");
fclose(Pos_Vel);

control_U = fopen('controller_u.txt','w');
fprintf(control_U, "time  , u_vx  , u_omega  \n");
fclose(control_U);

%  写入数据
for i=1:size(tout,1)
ERROR = fopen('error.txt','a');
fprintf(ERROR,'%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',error(i,:));
fclose(ERROR);  

Pos_Vel = fopen('pos_vel.txt','a');
fprintf(Pos_Vel,'%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',pos_vel(i,:));
fclose(Pos_Vel);  

control_U = fopen('controller_u.txt','a');
fprintf(control_U,'%.6f  ,  %.6f,  %.6f  \n',controller_u(i,:));
fclose(control_U); 
end
