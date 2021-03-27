% ʱ�䣺8.3
% mpc+smc

clear all
% close all
clc 

%% ����simulink
sim('sim20_8_3.slx'); % ���붯��ѧģ�͵Ŀ���

%% �Խ����ͼ
figure(1)   % λ��ͼ
subplot(2,2,1)
plot(body_position(:,1),body_position(:,2),'r-',ref_q(:,1),ref_q(:,2),'b-');
xlabel('X��');
ylabel('Y��');
title('�������˶�·�� ');
legend('ʵ��λ��','�ο�λ��');
hold on

subplot(2,2,2)   %x����
plot(tout,body_position(:,1),'r-',tout,ref_q(:,1),'b-');
xlabel('ʱ��');
ylabel('x������');
title('������x�᷽������仯 ');
legend('ʵ������','�ο�����');
hold on

subplot(2,2,3)   %y����
plot(tout,body_position(:,2),'r-',tout,ref_q(:,2),'b-');
xlabel('ʱ��');
ylabel('y������');
title('������y�᷽������仯 ');
legend('ʵ������','�ο�����');
hold on

subplot(2,2,4)   %��ڽ�
plot(tout,body_position(:,3),'r-',tout,ref_q(:,3),'b-');
xlabel('ʱ��');
ylabel('��ڽ�');
title('�����˺�ڽǱ仯 ');
legend('ʵ�ʽǶ�','�ο��Ƕ�');

figure(2)
subplot(2,2,1) 
e_v=body_vel(:,1)-ref_vel(:,1);
e_omega=body_vel(:,3)-ref_vel(:,3);
plot(tout,e_v,'r',tout,e_omega,'b');
xlabel('ʱ��');
ylabel('���');
title('�������ٶ����仯 ');
legend('�ٶ����','ת�����');

subplot(2,2,2)
plot(tout,control_u(:,1),'r',tout,ref_vel(:,1),'b');
xlabel('ʱ��');
ylabel('����');
title('�������ٶȱ仯 ');
legend('ʵ���ٶ�','�ο��ٶ�');

subplot(2,2,3)
plot(tout,control_u(:,2),'r',tout,ref_vel(:,3),'b');
xlabel('ʱ��');
ylabel('ת���ٶ�');
title('������ת����ٶȱ仯 ');
legend('ʵ��ת����ٶ�','�ο�ת����ٶ�');

subplot(2,2,4)
e_pos=body_position-ref_q;
plot(tout,e_pos(:,1),'r',tout,e_pos(:,2),'b',tout,e_pos(:,3),'g')
xlabel('ʱ��');
ylabel('���ֵ');
title('������λ�����仯 ');
legend('x��','y��','alpha��');

% �Խ����ͼ������ѧ��
figure(3)
subplot(2,1,1) 
plot(tout,control_u(:,1),'r',tout,ref_f(:,1),'b');
xlabel('ʱ��');
ylabel('����');
title('���������ǣ�����仯 ');
legend('ʵ��ǣ����','�ο�ǣ����');

subplot(2,1,2)
plot(tout,control_u(:,2),'r',tout,ref_f(:,2),'b');
xlabel('ʱ��');
ylabel('����');
title('�������Ҳ�ǣ�����仯 ');
legend('ʵ��ǣ����','�ο�ǣ����');


%% 
error=[tout e_pos e_v e_omega];
pos_vel=[tout ref_q body_position ref_vel body_vel];
controller_u=[tout control_u];

% % ����ļ� 
% ����
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
% %  д������
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
