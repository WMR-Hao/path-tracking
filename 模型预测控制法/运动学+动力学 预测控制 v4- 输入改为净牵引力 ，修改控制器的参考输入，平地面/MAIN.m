%  ��������� star up
%  2020 11 29 
clc
close all
clear all

%% ��ʼ��
dt = 0.032 ;    T = 32*1.5*1*0.7 ;    t = 0:dt:T;% T = 32*1*0.7 ;
X = 1 ;   Y = 0;    PSI = pi/2;  % ��ʼλ��    [1 0 pi/4];
vx = 0.1;  vy = 0;    gamma=0;
alpha=deg2rad(0);

wheel_r = 0.11;
gamma_max = 15;  % 6.4

Fdpl=0;  % ��ǣ����
Fdpr=0;

% vx1=(0.5*ones(size(t,2),1))';
% gamma1=sin(t);

%  Faa=1*sin(t);
% % Taa=(6.5*ones(size(t,2),1))';% 6.4746
%  Fbb=(0*ones(size(t,2),1))';

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

figure(10)   % λ��ͼ
plot(ref_position(:,1),ref_position(:,2),'m-');
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
          (body_pos(i,:),body_vel(i,:),ref_position(i+1:end,:),ref_velocity(i+1:end,:),Fdpl,Fdpr,dt,alpha)
 
    Fdpl=u_out(1);
    Fdpr=u_out(2);
 
%% ִ������ֵ    ģ��������˶�
    [X,Y,PSI,vx,vy,gamma,beta]=dynamics_model(Fdpl,Fdpr,X,Y,PSI,vx,vy,gamma,dt,alpha);

%%  �������
error_pos(i,1) = X - ref_position(i+1,1);
error_pos(i,2) = Y - ref_position(i+1,2);
error_pos(i,3) = PSI - ref_position(i+1,3);

error_vel(i,1) = vx - vxr;
error_vel(i,2) = vy - vyr;
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

%% 



%     % �����켣ͼ��
    if i>2
        figure(10)
        plot([body_pos(i-1,1) body_pos(i,1)],[body_pos(i-1,2) body_pos(i,2)],'b.-'); %'black.-'  'red.-'
        xlabel('X�� m');
        ylabel('Y�� m');
        title('�켣ͼ');
        axis equal
        hold on
        
% %         figure(11)
%         plot([t(i-1) t(i)],[body_vel(i-1,1) body_vel(i,1)],'black.-',...
%              [t(i-1) t(i)],[body_vel(i-1,2) body_vel(i,2)],'red.-',...
%              [t(i-1) t(i)],[body_vel(i-1,3) body_vel(i,3)],'green.-'); %'black.-'  'red.-'
%         xlabel('X�� m');
%         ylabel('Y�� m');
%         title('�ٶ�ͼ');
%         legend('�����ٶ�','�����ٶ�','ת�����ٶ�');
%         hold on
% 
% %         figure(2) %λ����� 
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


% % %%  
    
    i
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

figure(14)   % λ��ͼ
% subplot(2,2,1)
plot(body_pos(:,1),body_pos(:,2),'.-',ref_position(:,1),ref_position(:,2),'r-');
xlabel('X��');
ylabel('Y��');
title('������·�� '); 
legend('ʵ��λ��','�ο�λ��');
hold on


figure(15)   % �ٶ�ͼ
subplot(2,1,1)
plot(t,body_vel(:,1),'.-',tref,ref_velocity(2:end,1),'r-');
xlabel('X��');
ylabel('Y��');
title('�������ٶ� '); 
legend('ʵ��ǰ���ٶ�','�ο�ǰ���ٶ�');
hold on

subplot(2,1,2)
plot(t,body_vel(:,3),'.-',tref,ref_velocity(2:end,3),'r-');
xlabel('X��');
ylabel('Y��');
title('�������ٶ� '); 
legend('ʵ��ת����ٶ�','�ο�ת����ٶ�');
hold on

figure(16)
plot(t,BETA_vector(:,1),'r--',t,BETA_vector(:,2),'b',t,BETA_vector(:,3),'black',t,BETA_vector(:,4),'g',t,BETA_vector(:,5),'m');
xlabel('X��');
ylabel('Y��');
title('������beta�� '); 
legend('����','1��','2��','3��','4��');
hold on;


