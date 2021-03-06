% 2020 7 21
% ���ڶ���ѧģ�͵�ģ��Ԥ����� 
% �����˲������ļ�����pdf�е��ļ�
% 

clear all
% close all
clc 

%% ���з���
 sim('sim2020_7_21.slx'); % ����ѧģ�� mpc

%%  �Խ����ͼ
figure(1)   % λ��ͼ
subplot(2,2,1)
plot(body_position(:,1),body_position(:,2),'r-',ref_q(:,1),ref_q(:,2),'b-');
xlabel('X��');
ylabel('Y��');
title('�����˹켣');
legend('ʵ��','�ο�');
hold on

subplot(2,2,2) %x����
plot(tout,body_position(:,1),'r-',tout,ref_q(:,2),'b-');
xlabel('ʱ��');
ylabel('�仯��');
title('������x���� ');
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
ylabel('ƫ����');
title('������ƫ���Ǳ仯 ');
legend('ʵ�ʽǶ�','�ο��Ƕ�');

figure(2)
subplot(2,2,1) 
e_vx=body_vel(:,1)-ref_vel(:,1);
e_vy=body_vel(:,2)-ref_vel(:,2);
e_omega=body_vel(:,3)-ref_vel(:,3);
plot(tout,e_vx,'r',tout,e_vy,'g',tout,e_omega,'b');
xlabel('ʱ��');
ylabel('���');
title('�������ٶ����仯 ');
legend('�����ٶ����','�����ٶ����','ת�����');

subplot(2,2,2)
plot(tout,body_vel(:,1),'r',tout,ref_vel(:,1),'b');
xlabel('ʱ��');
ylabel('����');
title('�����������ٶȱ仯 ');
legend('ʵ�������ٶ�','�ο������ٶ�');

subplot(2,2,3)
plot(tout,body_vel(:,2),'r',tout,ref_vel(:,2),'b');
xlabel('ʱ��');
ylabel('����');
title('�����˺����ٶȱ仯 ');
legend('ʵ�ʺ����ٶ�','�ο������ٶ�');

subplot(2,2,4)
plot(tout,control_u(:,2),'r',tout,ref_vel(:,3),'b');
xlabel('ʱ��');
ylabel('ת���ٶ�');
title('������ת����ٶȱ仯 ');
legend('ʵ��ת����ٶ�','�ο�ת����ٶ�');

figure(3)
subplot(2,2,1)
e_pos=body_position-ref_q;
plot(tout,e_pos(:,1),'r',tout,e_pos(:,2),'b',tout,e_pos(:,3),'g')
xlabel('ʱ��');
ylabel('���ֵ');
title('������λ�����仯 ');
legend('x��','y��','alpha��');

% �Խ����ͼ������ѧ��
subplot(2,2,2) 
plot(tout,control_u(:,1),'r',tout,ref_f(:,1),'b');
xlabel('ʱ��');
ylabel('����');
title('���������ǣ�����仯 ');
legend('ʵ��ǣ����','�ο�ǣ����');

subplot(2,2,3)
plot(tout,control_u(:,2),'r',tout,ref_f(:,2),'b');
xlabel('ʱ��');
ylabel('����');
title('�������Ҳ�ǣ�����仯 ');
legend('ʵ��ǣ����','�ο�ǣ����');


% %% 
% error=[tout e_pos e_vx e_omega];
% pos_vel=[tout ref_q body_position ref_vel body_vel];
% controller_u=[tout control_u];
% 
% %% ����ļ� 
% % ����
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
% 
% % 
% %%  �Խ����ͼ������ѧMPC��
% 
% figure(1)   % λ��ͼ
% subplot(2,2,1)
% plot(body_position(:,1),body_position(:,2),'r-',ref_q(:,1),ref_q(:,2),'b-');
% xlabel('X��');
% ylabel('Y��');
% title('������·�� ');
% legend('ʵ��λ��','�ο�λ��');
% hold on
% 
% subplot(2,2,2)   %����
% plot(tout,body_vel(:,1),'r-',tout,ref_vel(:,1),'b-');
% xlabel('ʱ��');
% ylabel('����');
% title('�����˳��ٱ仯 ');
% legend('ʵ�ʳ���','�ο�����');
% hold on
% 
% subplot(2,2,3)   %ת�����ٶ�
% plot(tout,body_vel(:,3),'r-',tout,ref_vel(:,3),'b-');
% xlabel('ʱ��');
% ylabel('����');
% title('������ת���ٶȱ仯 ');
% legend('ʵ��ת��','�ο�ת��');
% hold on
% 
% subplot(2,2,4)   %���
% e_x=body_position(:,1)-ref_q(:,1);
% e_y=body_position(:,2)-ref_q(:,2);
% e_alpha=body_position(:,3)-ref_q(:,3);
% plot(tout,e_x,'r-',tout,e_pos(),'b',tout,e_alpha,'g',tout,zeros(1,size(tout,1)),'--');
% xlabel('ʱ��');
% ylabel('���');
% title('���������仯 ');
% legend('x�������','y�������','ת�����');
% 
% figure(2)
% subplot(2,2,1) 
% e_v=body_vel(:,1)-ref_vel(:,1);
% e_omega=body_vel(:,3)-ref_vel(:,3);
% plot(tout,e_v,'r',tout,e_omega,'b');
% xlabel('ʱ��');
% ylabel('���');
% title('�������ٶ����仯 ');
% legend('�ٶ����','ת�����');
% 
% subplot(2,2,2) 
% plot(tout,control_u(:,1),'r',tout,ref_f(:,1),'b');
% xlabel('ʱ��');
% ylabel('����');
% title('���������ǣ�����仯 ');
% legend('ʵ��ǣ����','�ο�ǣ����');
% 
% subplot(2,2,3)
% plot(tout,control_u(:,2),'r',tout,ref_f(:,2),'b');
% xlabel('ʱ��');
% ylabel('����');
% title('�������Ҳ�ǣ�����仯 ');
% legend('ʵ��ǣ����','�ο�ǣ����');
% 
% subplot(2,2,4)
% plot(tout,e_pos(:,1),'r',tout,e_pos(:,2),'b',tout,e_pos(:,3),'g')
% xlabel('ʱ��');
% ylabel('���ֵ');
% title('������λ�����仯 ');
% legend('x��','y��','alpha��');
% 


%% 
error=[tout e_pos e_vx e_vy e_omega];
pos_vel=[tout ref_q body_position ref_vel body_vel];
controller_u=[tout control_u];

%% ����ļ� 
% ����
ERROR = fopen('error.txt','w');
fprintf(ERROR, "time  , ex  , ey  , ealpha  , evx ,  evy , eomega  \n");
fclose(ERROR);

Pos_Vel = fopen('pos_vel.txt','w');
fprintf(Pos_Vel, "time  , ref_x  , ref_y  , ref_alpha  , x  , y  , alpha  , ref_vx  , ref_vy  , ref_omega  , vx  , vy  , omega  \n");
fclose(Pos_Vel);

control_U = fopen('controller_u.txt','w');
fprintf(control_U, "time  , u_vx  , u_omega  \n");
fclose(control_U);

%  д������
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
