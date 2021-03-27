%  ��������� star up
%  2020 11 29 
clc
clear all

%% ��ʼ��
dt=0.01;    T=20;  
t=0:dt:T;
X=0 ;   Y=0;    PHI=deg2rad(pi/4);  % ��ʼλ��
vx=0.1;      gamma=0.1;        vy=0;

wheel_r = 0.11;
gamma_max = 15; % 6.4

% vx1=(0.5*ones(size(t,2),1))';
% gamma1=sin(t);

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
ref_position(1,:)=[0 0 PHI];
vxr=1;
vyr=0;
% gammar=0.5;
gammar=0.5;

for i=1:length(t)
    [Xr,Yr,ALPHAr] = ref_path(vxr,gammar,dt,ref_position(i,1),ref_position(i,2),ref_position(i,3));
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

error_pos = zeros(length(t),3);
error_vel = zeros(length(t),3);

%% ����
% u=zeros(length(t),4);% u=[vx omega omega_l omega_r]

for i=1:length(t)

    
    %% ��������ֵ
    body_pos(i,1)= X ;    % body_pos=[X Y PSI]
    body_pos(i,2)= Y ;
    body_pos(i,3)= PHI ;
    body_vel(i,1)= vx ;   %body_vel=[vx vy gamma]
    body_vel(i,2)= vy ;
    body_vel(i,3)= gamma ;
    
    %% ������ ����
    %tic
%     %************************   ����������  *************************
     u_out = my_kin_MPC_controller(body_pos(i,:),body_vel(i,:),ref_position(i,:),vxr,gammar,dt);

% u_out=back_stepping(body_pos(i,1),ref_position(i+1,1),body_pos(i,2),ref_position(i+1,2),body_pos(i,3),ref_position(i+1,3),vxr,gammar);
    vx=u_out(1);
    gamma=u_out(2);
%   *********   ���������� end
    %toc

%% ִ������ֵ    ģ��������˶�
    [X,Y,PHI,vx,gamma]=kinematics_model(vx,gamma,X,Y,PHI,dt);
    
%%  �������
error_pos(i,1) = X - ref_position(i,1);
error_pos(i,2) = Y - ref_position(i,2);
error_pos(i,3) = PHI - ref_position(i,3);

error_vel(i,1) = vx - vxr;
error_vel(i,2) = vy - vyr;
error_vel(i,3) = gamma - gammar;


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
    drawnow
end

%% ��������ã�
% %% ��ͼ

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

figure(5)   % λ��ͼ
% subplot(2,2,1)
plot(t, body_pos(:,3),'blue.-');
xlabel('X��');
ylabel('Y��');
title('������·�� ');
legend('ʵ��λ��');
hold on
axis equal





