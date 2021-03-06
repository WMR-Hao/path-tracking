%  ��������� star up
%  2020 11 29 
clc
close all
clear all

%% ��ʼ��
dt=0.05;    T=20;   t=0:dt:T;
X=1 ;   Y=0;    PSI = pi/4;  % ��ʼλ��    [1 0 pi/4];
vx=0.1;  vy=0;    gamma=0;
alpha=deg2rad(0);

wheel_r = 0.11;
gamma_max = 15; % 6.4

Ta=6.4746;
Tb=0;

Ta0=6.4746;
Tb0=0;

% vx1=(0.5*ones(size(t,2),1))';
% gamma1=sin(t);

% Taa=0.5*sin(t);
% Taa=(6.5*ones(size(t,2),1))';% 6.4746
% Tbb=(0*ones(size(t,2),1))';

%% ������¼���ݵ�����ļ�  
% ���� ��Ҫ�ȴ���simdata�ļ���
ERROR = fopen('simdata\error.txt','w');
fprintf(ERROR, "time  , ex  , ey  , ealpha  , evx ,  evy , eomega  \n");
fclose(ERROR);

Pos_Vel = fopen('simdata\pos_vel.txt','w');
fprintf(Pos_Vel, "time  , ref_x  , ref_y  , ref_alpha  , x  , y  , alpha  , ref_vx  , ref_vy  , ref_omega  , vx  , vy  , omega  \n");
fclose(Pos_Vel);

control_U = fopen('simdata\controller_u.txt','w');
fprintf(control_U, "time  , u_vx  , u_omega  \n");
fclose(control_U);

%% �ο��켣����
ref_position=zeros(length(t),3);
ref_position(1,:)=[1 0 pi/4];
vxr=1;
vyr=0;
% gammar=0.5;
gammar=0;

for i=1:length(t)
    [Xr,Yr,ALPHAr] = ref_path(vxr,gammar,dt,ref_position(i,1),ref_position(i,2),ref_position(i,3));
    ref_position(i+1,1)=Xr;
    ref_position(i+1,2)=Yr;
    ref_position(i+1,3)=ALPHAr;
    ref_velocity(i+1,1)=vxr;
    ref_velocity(i+1,2)=vyr;
    ref_velocity(i+1,3)=gammar;
end

figure(10)   % λ��ͼ
plot(ref_position(:,1),ref_position(:,2),'r-');
hold on

%% �洢λ�����ݺ��ٶ�����
body_pos=zeros(length(t),3);
body_vel=zeros(length(t),3);

error_pos = zeros(length(t),3);
error_vel = zeros(length(t),3);

BETA_vector=zeros(length(t),5);

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
    %tic
%     %************************   ����������  *************************
      u_out = my_dyn_MPC_controller...
          (body_pos(i,:),body_vel(i,:),ref_position(i,:),ref_velocity(i,:),Ta,Tb,Ta0,Tb0,dt,alpha);

%     uu(i,:)=u_out';
% 
    Ta=u_out(1);
    Tb=u_out(2);
    
%     Ta=Taa(i);
%     Tb=Tbb(i);
%     
    
    
%   *********   ���������� end
    %toc
%     u(i,1)=vx;
%     u(i,2)=gamma;
% vx=vx1(i);
% gamma=gamma1(i);
%     [omega_l,omega_r] = velocity_resolution(vx,omega);  % ��[v  w]->[wl wr]
%     omega_wheel=[omega_l  omega_r];
%     omega_wheel=limitvel(omega_wheel,omega_max); % ���ݻ�����ʵ���������omega �޷�
%     omega_l=omega_wheel(1);    omega_r=omega_wheel(2) ;
%     u(i,3)=omega_l;    u(i,4)=omega_r;

%% ִ������ֵ    ģ��������˶�
    [X,Y,PSI,vx,vy,gamma,beta]=dynamics_model(Ta,Tb,X,Y,PSI,vx,vy,gamma,dt,alpha);

%%  �������
error_pos(i,1) = X - ref_position(i,1);
error_pos(i,2) = Y - ref_position(i,2);
error_pos(i,3) = PSI - ref_position(i,3);

error_vel(i,1) = vx - vxr;
error_vel(i,2) = vy - vy;
error_vel(i,3) = gamma - gammar;

% ��¼���ֲ�ƫ��
BETA_vector(i,:)=beta;
%% �洢������simdata�ļ�
% ERROR = fopen('error.txt','a');
% fprintf(ERROR,'%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',error(i,:));
% fclose(ERROR);  

% Pos_Vel = fopen('pos_vel.txt','a');
% fprintf(Pos_Vel,'%.6f  ,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f,  %.6f\n',pos_vel(i,:));
% fclose(Pos_Vel);  

% control_U = fopen('controller_u.txt','a');
% fprintf(control_U,'%.6f  ,  %.6f,  %.6f  \n',controller_u(i,:));
% fclose(control_U);

    %% �����켣ͼ��
    if i>2
        figure(10)
        plot([body_pos(i-1,1) body_pos(i,1)],[body_pos(i-1,2) body_pos(i,2)],'b.-'); %'black.-'  'red.-'
        xlabel('X�� m');
        ylabel('Y�� m');
        title('�켣ͼ');
        axis equal
        hold on
        
        figure(11)
        plot([t(i-1) t(i)],[body_vel(i-1,1) body_vel(i,1)],'black.-',...
             [t(i-1) t(i)],[body_vel(i-1,2) body_vel(i,2)],'red.-',...
             [t(i-1) t(i)],[body_vel(i-1,3) body_vel(i,3)],'green.-'); %'black.-'  'red.-'
        xlabel('X�� m');
        ylabel('Y�� m');
        title('�ٶ�ͼ');
        legend('�����ٶ�','�����ٶ�','ת�����ٶ�');
        hold on

%         figure(2) %λ����� 
%         plot( [t(i-1) t(i)], [error_pos(i-1,1) error_pos(i,1)],'blue.-',...
%               [t(i-1) t(i)], [error_pos(i-1,2) error_pos(i,2)],'red.-',...
%               [t(i-1) t(i)], [error_pos(i-1,3) error_pos(i,3)],'green.-');
%         hold on
%         
%         figure(3) % �ٶ����
%         plot( [t(i-1) t(i)], [error_vel(i-1,1) error_vel(i,1)],'blue.-',...
%               [t(i-1) t(i)], [error_vel(i-1,2) error_vel(i,2)],'red.-',...
%               [t(i-1) t(i)], [error_vel(i-1,3) error_vel(i,3)],'green.-');
%         hold on
%  
    end
    drawnow
    i;
end

%% ��������ã�
% %% ��ͼ

        figure(12) %λ����� 
        plot( t, error_pos(:,1),'blue.-',...
              t, error_pos(:,2),'red.-',...
              t, error_pos(:,3),'green.-');
        hold on
        xlabel('ʱ�� t');
        ylabel('λ�� ');
        title('λ�����ͼ');
        legend('X�������','Y�������','ת���Ƕ����');
       
        
        figure(13) % �ٶ����
        plot( t, error_vel(:,1),'blue.-',...
              t, error_vel(:,2),'red.-',...
              t, error_vel(:,3),'green.-');
        hold on
        xlabel('ʱ�� t');
        ylabel('�ٶ�  ');
        title('�ٶ����ͼ');
        legend('�����ٶ����','�����ٶ����','ת�����ٶ����');




% figure(2)   % λ��ͼ
% % subplot(2,2,1)
% plot(body_pos(:,1),body_pos(:,2),'.-',ref_position(:,1),ref_position(:,2),'r-');
% xlabel('X��');
% ylabel('Y��');
% title('������·�� ');
% legend('ʵ��λ��','�ο�λ��');
% hold on




