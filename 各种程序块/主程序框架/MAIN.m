%  ��������� star up
%  2020 11 29 

clc
clear all

%% ��ʼ��
dt=0.001;    T=1;   t=0:dt:T;
X=2 ;   Y=0;    ALPHA=pi/2;  % ��ʼλ��
vx=0.0001;  vy=0;    omega=0;

wheel_r = 0.11;
omega_max = 15; % 6.4

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
ref_position(1,:)=[0 0 pi/4];
vxr=1;
omegar=0.5;

for i=1:length(t)
    [Xr,Yr,ALPHAr] = ref_path(vxr,omegar,dt,ref_position(i,1),ref_position(i,2),ref_position(i,3));
    ref_position(i+1,1)=Xr;
    ref_position(i+1,2)=Yr;
    ref_position(i+1,3)=ALPHAr;
end

figure(1)   % λ��ͼ
plot(ref_position(:,1),ref_position(:,2),'r-');
hold on

%% �洢λ�����ݺ��ٶ�����
body_pos=zeros(length(t),3);
body_vel=zeros(length(t),3);

%% ����
% u=zeros(length(t),4);% u=[vx omega omega_l omega_r]

for i=1:length(t)
    %% ��������ֵ
    body_pos(i,1)=X;
    body_pos(i,2)=Y;
    body_pos(i,3)=ALPHA;
    body_vel(i,1)=vx;
    body_vel(i,2)=vy;
    body_vel(i,3)=omega;
    
    %% ������ ����
    %tic
    %************************   ����������  *************************
    
    %*********   ���������� end
    %toc
    u(i,1)=vx;
    u(i,2)=omega;
    
%     [omega_l,omega_r] = velocity_resolution(vx,omega);  % ��[v  w]->[wl wr]
%     omega_wheel=[omega_l  omega_r];
%     omega_wheel=limitvel(omega_wheel,omega_max); % ���ݻ�����ʵ���������omega �޷�
%     omega_l=omega_wheel(1);    omega_r=omega_wheel(2) ;
%     u(i,3)=omega_l;    u(i,4)=omega_r;

Ta=13;% 4.6617
Tb=10;
gamma=0;
% [FL,FR]=Torque_resolution(Fa,Fb);
    
    %% ִ������ֵ    ģ��������˶�
    [X,Y,ALPHA,vx,vy,omega]=dynamics_model(Ta,Tb,X,Y,ALPHA,vx,vy,omega,dt,gamma);
    
    %% �����켣ͼ��
    if i>2
        plot([body_pos(i-1,1) body_pos(i,1)],[body_pos(i-1,2) body_pos(i,2)],'black.-');
   
        hold on
    end
    drawnow
    i
end

gamma=0;
[TL,TR]  =Torque_resolution(Ta,Tb)

axis equal
%% ��������ã�


% %% ��ͼ
% figure(2)   % λ��ͼ
% % subplot(2,2,1)
% plot(body_pos(:,1),body_pos(:,2),'.-',ref_position(:,1),ref_position(:,2),'r-');
% xlabel('X��');
% ylabel('Y��');
% title('������·�� ');
% legend('ʵ��λ��','�ο�λ��');
% hold on


