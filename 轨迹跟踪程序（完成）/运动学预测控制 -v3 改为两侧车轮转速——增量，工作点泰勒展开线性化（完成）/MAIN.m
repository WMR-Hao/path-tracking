%  ��������� star up
%  2020 11 29 
clc
clear all
% close all

%% ��ʼ��
dt = 0.032;    T =  30  ;  
t = 0:dt:T;
X = 0 ;   Y = 0.5 ;    PSI=deg2rad(0);  % ��ʼλ��
vx = 0;      gamma=0;        vy = 0;% - 0.05

Np = 30 ;  % 40 �� 20 �������� 
Nc = 10 ;  

%%   ��������     �ȷ�3at ���̲���
global  m Iz g alpha wheel_r l d c_wheel  Hight
wheel_r = 0.11;
d = 0.197;                  l = 0.1331/2; % H = 0.2455; 
wheel_width = 0.095;
mass_chassis = 8.8;            mass_wheel = 0.8;
g  = 9.81 ;
m = ( mass_chassis + mass_wheel * 4 ) ;
Iz = 2 ;   
c_wheel = 50 ;
Hight = 0.5 ;
alpha =  deg2rad( 0 );  % �¶Ƚ�  


%% ������¼���ݵ�����ļ�  
% ���� ��Ҫ�ȴ���simdata�ļ���
ERROR = fopen('simdata\error.txt','w');
fprintf(ERROR, "time  , ex  , ey  , ealpha  , evx ,  evy , eomega  \n");
fclose(ERROR);

Pos_Vel = fopen('simdata\pos_vel.txt','w');
fprintf(Pos_Vel, "time  , ref_x  , ref_y  , ref_alpha  , x  , y  , alpha  , ref_vx  , ref_vy  , ref_omega  , vx  , vy  , omega  \n");
fclose(Pos_Vel);

control_U = fopen('simdata\controller_u.txt','w');
fprintf(control_U, "time  , u_omega_l  , u_omega_r  \n");
fclose(control_U);

%% �ο��켣����
tref=0:dt:T+Np*dt;

ref_position=zeros(length(tref),3);
ref_position(1,:)=[0 0 0];
vyr = 0;

 vxr  =  0.5  *  ones(size(tref));
 gammar  = 0.2   *  ones(size(tref));


for i = 1 :length(tref)
    [Xr,Yr,ALPHAr] = ref_path...
        (vxr(i),gammar(i),dt,ref_position(i,1),ref_position(i,2),ref_position(i,3));
    ref_position(i+1,1) = Xr;
    ref_position(i+1,2) = Yr;
    ref_position(i+1,3) = ALPHAr;
end

figure(1)   % λ��ͼ
plot(ref_position(:,1),ref_position(:,2),'r-');
hold on

%% �洢λ�����ݺ��ٶ�����
body_pos = zeros(length(t),3);
body_vel = zeros(length(t),3);

error_pos = zeros(length(t),3);
error_vel = zeros(length(t),3);

%% ����
% u=zeros(length(t),4);% u=[vx omega omega_l omega_r]

for i=1:length(t)
    %% ��������ֵ
    body_pos(i,1)= X ;    % body_pos=[X Y PSI]
    body_pos(i,2)= Y ;
    body_pos(i,3)= PSI ;
    body_vel(i,1)= vx ;   %body_vel=[vx vy gamma]
    body_vel(i,2)= vy ;
    body_vel(i,3)= gamma ;
    
%% ������ ����
    %************************   ����������  *************************
     u_out = my_kin_MPC_controller_v3_delta_omega...
         (body_pos(i,:),body_vel(i,:),ref_position(i+1:end,:),dt,Np,Nc) 
     
    omega_l = u_out(1);
    omega_r = u_out(2);
    
%% ִ������ֵ  ���Ҳ೵�ֵ��   ģ��������˶�
    [X,Y,PSI,vx,gamma] = kinematics_model_v2(omega_l,omega_r,X,Y,PSI,dt,vy);
    
%%  �������

error_pos(i,1) = X - ref_position(i+1,1);  
error_pos(i,2) = Y - ref_position(i+1,2);
error_pos(i,3) = PSI - ref_position(i+1,3);

error_vel(i,1) = vx - vxr(i);
error_vel(i,2) = vy - vyr;
error_vel(i,3) = gamma - gammar(i);
%% 
error(i,:) = [t(i) error_pos(i,:) error_vel(i,:)];
pos_vel(i,:) = [t(i) ref_position(i,:)  body_pos(i,:) vxr(i) vyr gammar(i)  body_vel(i,:)];

%% �洢������simdata�ļ�
ERROR = fopen('simdata\error.txt','a');
fprintf(ERROR, '%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',error(i,:));
fclose(ERROR);  

Pos_Vel = fopen('simdata\pos_vel.txt','a');
fprintf(Pos_Vel,'%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',pos_vel(i,:));
fclose(Pos_Vel);  

control_U = fopen('simdata\controller_u.txt','a');
fprintf(control_U,'%.6f  ,  %.6f,  %.6f  \n',t(i),u_out');
fclose(control_U);

% %% �����켣ͼ��




%     if i>2
%         figure(1)
%         plot([body_pos(i-1,1) body_pos(i,1)],[body_pos(i-1,2) body_pos(i,2)],'m.-'); %'black.-'  'red.-'
%         axis equal
%         hold on
% 
% %         figure(2) %λ����� 
% %         plot( [t(i-1) t(i)], [error_pos(i-1,1) error_pos(i,1)],'blue.-',...
% %               [t(i-1) t(i)], [error_pos(i-1,2) error_pos(i,2)],'red.-',...
% %               [t(i-1) t(i)], [error_pos(i-1,3) error_pos(i,3)],'green.-');
% %         hold on
% %         
% %         figure(3) % �ٶ����
% %         plot( [t(i-1) t(i)], [error_vel(i-1,1) error_vel(i,1)],'blue.-',...
% %               [t(i-1) t(i)], [error_vel(i-1,2) error_vel(i,2)],'red.-',...
% %               [t(i-1) t(i)], [error_vel(i-1,3) error_vel(i,3)],'green.-');
% %         hold on
% %  
%     end
%     drawnow
    i;




end

%% ��������ã�
% %% ��ͼ
% close all

figure(2) %λ�����
plot( t, error_pos(:,1),'blue.-',...
    t, error_pos(:,2),'red.-',...
    t, error_pos(:,3),'green.-');
hold on
xlabel('ʱ�� t');
ylabel('λ�� ');
legend('x���','y�����','�Ƕ����');
title('λ�����ͼ');

figure(3) % �ٶ����
plot( t, error_vel(:,1),'blue.-',...
    t, error_vel(:,2),'red.-',...
    t, error_vel(:,3),'green.-');
hold on
legend('�����ٶ����','�����ٶ����','ת�����ٶ����');
xlabel('ʱ�� t');
ylabel('�ٶ�  ');
title('�ٶ����ͼ');



figure(4)   % λ��ͼ
% subplot(2,2,1)
plot(body_pos(:,1),body_pos(:,2),'b.-',ref_position(:,1),ref_position(:,2),'r-');
xlabel('X��');
ylabel('Y��');
title('������·�� ');
legend('ʵ��λ��','�ο�λ��');
hold on
axis equal

figure(5) %λ�����
plot( t, body_vel(:,1),'blue.-',...
    t, body_vel(:,2),'red.-',...
    t, body_vel(:,3),'green.-');
hold on
xlabel('ʱ�� t');
ylabel('�ٶ� ');
legend('�����ٶ�','�����ٶ�','ת�����ٶ�');
title('�ٶȱ仯ͼ');




