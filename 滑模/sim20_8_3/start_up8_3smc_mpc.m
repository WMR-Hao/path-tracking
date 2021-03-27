% 时间：8.3
% mpc+smc

clear all
% close all
clc 

%% 调用simulink
sim('sim20_8_3.slx'); % 加入动力学模型的控制

%% 对结果作图
figure(1)   % 位置图
subplot(2,2,1)
plot(body_position(:,1),body_position(:,2),'r-',ref_q(:,1),ref_q(:,2),'b-');
xlabel('X轴');
ylabel('Y轴');
title('机器人运动路径 ');
legend('实际位置','参考位置');
hold on

subplot(2,2,2)   %x方向
plot(tout,body_position(:,1),'r-',tout,ref_q(:,1),'b-');
xlabel('时间');
ylabel('x轴坐标');
title('机器人x轴方向坐标变化 ');
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
ylabel('横摆角');
title('机器人横摆角变化 ');
legend('实际角度','参考角度');

figure(2)
subplot(2,2,1) 
e_v=body_vel(:,1)-ref_vel(:,1);
e_omega=body_vel(:,3)-ref_vel(:,3);
plot(tout,e_v,'r',tout,e_omega,'b');
xlabel('时间');
ylabel('误差');
title('机器人速度误差变化 ');
legend('速度误差','转动误差');

subplot(2,2,2)
plot(tout,control_u(:,1),'r',tout,ref_vel(:,1),'b');
xlabel('时间');
ylabel('车速');
title('机器人速度变化 ');
legend('实际速度','参考速度');

subplot(2,2,3)
plot(tout,control_u(:,2),'r',tout,ref_vel(:,3),'b');
xlabel('时间');
ylabel('转向速度');
title('机器人转向角速度变化 ');
legend('实际转向角速度','参考转向角速度');

subplot(2,2,4)
e_pos=body_position-ref_q;
plot(tout,e_pos(:,1),'r',tout,e_pos(:,2),'b',tout,e_pos(:,3),'g')
xlabel('时间');
ylabel('误差值');
title('机器人位姿误差变化 ');
legend('x轴','y轴','alpha角');

% 对结果作图（动力学）
figure(3)
subplot(2,1,1) 
plot(tout,control_u(:,1),'r',tout,ref_f(:,1),'b');
xlabel('时间');
ylabel('轮速');
title('机器人左侧牵引力变化 ');
legend('实际牵引力','参考牵引力');

subplot(2,1,2)
plot(tout,control_u(:,2),'r',tout,ref_f(:,2),'b');
xlabel('时间');
ylabel('轮速');
title('机器人右侧牵引力变化 ');
legend('实际牵引力','参考牵引力');


%% 
error=[tout e_pos e_v e_omega];
pos_vel=[tout ref_q body_position ref_vel body_vel];
controller_u=[tout control_u];

% % 输出文件 
% 标题
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
