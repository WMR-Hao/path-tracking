 % ����޸�ʱ�䣺3.1
% ���ڶ���ѧ ģ�� ��  MPC  ������
% u=[Ta ; Tb]
% ref=[Xr Yr PSIr vxr xyr gammar]

function [u_out]=my_dyn_MPC_controller(body_pos,body_vel,ref_position,ref_velocity,Ta_previous,Tb_previous,dt,alpha)
%    body_pos,body_vel              ��ǰtʱ�̵�λ�˺��ٶȡ�
%    ref_position,ref_velocity      ��ǰtʱ�̵Ĳο�λ�˺Ͳο��ٶȡ�
%    Ta,Tb                          ǰһʱ��t-1ʱ�̵����롣
%    Ta0,Tb0                       
%    dt                             ���沽��
%    alpha                          б�����
 
%  Ԥ��ʱ��Np	����ʱ��Nc
Nc = 2 ; 
Np = 20 ;

% ��ʼ��
body_pos = double(body_pos);            body_vel = double(body_vel); 
ref_position = double(ref_position);    ref_velocity = double(ref_velocity);


X = body_pos(1);    Y = body_pos(2);    PSI = body_pos(3);  % �����ƫ����A��B��ʱ����
vx=body_vel(1);     vy=body_vel(2);     gamma=body_vel(3);

%  Yref=[ref_position ref_velocity]';

ut=[Ta_previous; Tb_previous];    %ǰһʱ�� t-1 ����ʵ������������������Nu
Nu=size(ut,1);

qt=[body_pos';body_vel'];    %��ǰʱ�� t ��״̬����״̬������Nx
Nx=size(qt,1);

q = 1*[1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;];
R = 1*eye(Nu*Nc,Nu*Nc);

%% �ȷ�3at ������ ���̲��� 
d = 0.197;  l = 0.1331; % H = 0.2455; 
gravity = [0 0 9.81];
wheel_radius = 0.11;        wheel_width = 0.095;
mass_chassis = 8.8;            mass_wheel = 0.8;
g = gravity(3);
m = mass_chassis+mass_wheel*4;
Iz = 5.14 ;  Ix = 2.8305 ;  Iy = 4.6786;

c_wheel = 10;

%% ������ ����ѧģ�ͣ� 

%% �õ���ƹ켣
q_prediction_cell = cell(Np,1);
% q_prediction_cell{1,1}=[X;Y;PSI;vx;vy;gamma];
q_prediction_cell{1,1}=[qt];

for prediction_time=1:Np
    qt_prediction = q_prediction_cell{prediction_time,1};
    Xt=qt_prediction(1);   % k|tʱ�̵�״̬��
    Yt=qt_prediction(2);
    PSIt=qt_prediction(3);
    vxt=qt_prediction(4);
    vyt=qt_prediction(5);
    gammat=qt_prediction(6);
        
    Fat=ut(1);  % k|tʱ�̵������� ��ƹ켣�ϣ�����㶨��������
    Fbt=ut(2);
    
    % ��ƫ��
    beta1t = atan(( ( vyt+l*gammat )/( vxt-d*gammat ) ));    %beta1= ( vy+l*gamma )/( vx-d*gamma )
    beta2t = atan(( ( vyt+l*gammat )/( vxt+d*gammat ) ));
    beta3t = atan(( ( vyt-l*gammat )/( vxt+d*gammat ) ));
    beta4t = atan(( ( vyt-l*gammat )/( vxt-d*gammat ) ));
    
    beta_max=deg2rad(2.5);
    beta_min=deg2rad(-2.5);
    
%     betat=atan(vyt/vxt);  % ���Ĳ�ƫ��
    
    % ������
    F1yt=-c_wheel*beta1t;
    F2yt=-c_wheel*beta2t;
    F3yt=-c_wheel*beta3t;
    F4yt=-c_wheel*beta4t;
      
    if beta1t>=beta_max
        F1yt=-c_wheel*beta_max;
    end
    if beta1t<=beta_min
        F1yt=-c_wheel*beta_min;
    end
    %%
    if beta2t>=beta_max
        F2yt=-c_wheel*beta_max;
    end
    if beta2t<=beta_min
        F2yt=-c_wheel*beta_min;
    end
    %%
    if beta3t>=beta_max
        F3yt=-c_wheel*beta_max;
    end
    if beta3t<=beta_min
        F3yt=-c_wheel*beta_min;
    end
    %%
    if beta4t>=beta_max
        F4yt=-c_wheel*beta_max;
    end
    if beta4t<=beta_min
        F4yt=-c_wheel*beta_min;
    end
    
    Fyt=F1yt+F2yt+F3yt+F4yt;
    
%     %% ������
%     W=m*g/4;
%     W1=W;
%     W2=W;
%     W3=W;
%     W4=W;
%     
%     a=0.7;
%     b=0.36;
%     s=0.2;
%     f = ( W1*(a*s+b)+W2*(a*s+b)+W3*(a*s+b)+W4*(a*s+b) );
    % Fa = Ta/wheel_r - ( W1*(a*s+b)+W2*(a*s+b)+W3*(a*s+b)+W4*(a*s+b) );
    % Fb=Tb/wheel_r + (W1*(a*s+b)+W2*(a*s+b)-W3*(a*s+b)-W4*(a*s+b));
    
    %%  ���棬����������ѧģ��  ��ǣ�������
    vxtnext=( (Fat-m*g*sin(alpha)*sin(PSIt)+m*vyt*gammat)/m )*dt + vxt;
    vytnext=( (Fyt-m*g*sin(alpha)*cos(PSIt)-m*vxt*gammat)/m )*dt + vyt;
    gammatnext=( d*( Fbt )+l*(-F1yt-F2yt+F3yt+F4yt)) /Iz;
    
    % vynext=( (Fy-m*g*sin(alpha)*cos(PSI))/m )*dt + vy;  %-vx*gamma  ��������
    % vxnext=( (Fa-m*g*sin(alpha)*sin(PSI))/m )*dt + vx;  %+vy*gamma
    % gammanext=( d*( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l*(F1y+F2y-F3y-F4y)) /Iz;
    
    %%  ���棬��������Ϊ���������Ҳ��ƫ�ǲ�ͬ��ͬ���ƫ����ͬ������ѧģ��
    % F1y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
    % F2y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
    % F3y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
    % F4y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
    % Fy=F1y+F2y+F3y+F4y;
    %
    % vxnext=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
    % vynext=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
    % gammanext=( d( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l(F1y+F2y-F3y-F4y)) /Iz;
    
    % �˶�ѧģ��
    PSItnext = gammat*dt + PSIt;
    Xtnext = ( vxt*cos(PSIt)-vyt*sin(PSIt) )*dt+Xt;
    Ytnext = ( vxt*sin(PSIt)+vyt*cos(PSIt) )*dt+Yt;
    
    q_prediction_cell{prediction_time+1,1}=[Xtnext;Ytnext;PSItnext;vxtnext;vytnext;gammatnext];
    
end
q_prediction=cell2mat(q_prediction_cell);  % ��tʱ�̣�����������ʱ��ϵͳ��״̬�������۱仯 qt(k)...qt(k+Np)


%% ���ֲ�ƫ�ǵ�ƫ�� 
dbeta1_dvx = (-vy-l*gamma)/((vx-d*gamma)^2);    % beta1 �� ƫ����
dbeta1_dvy = 1/(vx-d*gamma);
dbeta1_dgamma = (l*vx+d*vy)/((vx-d*gamma)^2);

dbeta2_dvx = (-vy-l*gamma)/((vx+d*gamma)^2);    % beta2 �� ƫ����
dbeta2_dvy = 1/(vx+d*gamma);
dbeta2_dgamma = (l*vx-d*vy)/((vx+d*gamma)^2);

dbeta3_dvx = (-vy+l*gamma)/((vx+d*gamma)^2);    % beta3 �� ƫ����
dbeta3_dvy = 1/(vx+d*gamma);
dbeta3_dgamma = (-l*vx-d*vy)/((vx+d*gamma)^2);

dbeta4_dvx = (-vy+l*gamma)/((vx-d*gamma)^2);    % beta4 �� ƫ����
dbeta4_dvy = 1/(vx-d*gamma);
dbeta4_dgamma = (-l*vx+d*vy)/((vx-d*gamma)^2);

sum_dbeta_dvx=dbeta1_dvx+dbeta2_dvx+dbeta3_dvx+dbeta4_dvx; % ƫ�����ĺ�
sum_dbeta_dvy=dbeta1_dvy+dbeta2_dvy+dbeta3_dvy+dbeta4_dvy;
sum_dbeta_dgamma=dbeta1_dgamma+dbeta2_dgamma+dbeta3_dgamma+dbeta4_dgamma;

diff_dbeta_dvx = -dbeta1_dvx-dbeta2_dvx+dbeta3_dvx+dbeta4_dvx;  % ƫ�����Ĳ�
diff_dbeta_dvy = -dbeta1_dvy-dbeta2_dvy+dbeta3_dvy+dbeta4_dvy;
diff_dbeta_dgamma = -dbeta1_dgamma-dbeta2_dgamma+dbeta3_dgamma+dbeta4_dgamma;
%%
% ���ƾ������(����������ת��Ϊ��ɢ���󣬲��ý����㷨  A=I+T*A(t),B=T*B(t))
% �˴���delta_X=AX+BU ��������X=[a-r]=[X_I-Xr;Y_I-Yr;ALPHA_I-ALPHAr]statu_e=A*statu_e+B*u_e   %��ɢ���ģ��
    % OUT=delta_X=a-r
A=[ 0, 0, -vx*sin(PSI)-vy*cos(PSI), cos(PSI), -sin(PSI), 0;
    0, 0,  vx*cos(PSI)-vy*sin(PSI), sin(PSI),  cos(PSI), 0;
    0, 0,           0,                  0,      0,       1;
    0, 0, -g*sin(alpha)*cos(PSI),       0    gamma,    vy;
    0, 0,  g*sin(alpha)*sin(PSI), -(c_wheel/m)*(sum_dbeta_dvx)-gamma,  -(c_wheel/m)*sum_dbeta_dvy,  -(c_wheel/m)*sum_dbeta_dgamma-vx;
    0, 0,       0,                l*c_wheel*( diff_dbeta_dvx )/Iz,  l*c_wheel*(diff_dbeta_dvy)/Iz,  l*c_wheel*(diff_dbeta_dgamma)/Iz];

B =[0 0;
    0 0;
    0 0;
    1/(wheel_radius*m) 0;
    0 0;
    0 d/(wheel_radius*Iz)];

A1=A*dt+eye(Nx);  % �����е�A~��B~
B1=B*dt;
C = eye(Nx);

% �õ��µ�X=A_new*X+B_new*delta_u   X=[X;u-ur]

q_=[qt;ut];   %qt��ǰʱ�� t ��״̬��   ut ǰһʱ�� t-1 ����ʵ������������������Nu
A_ = [A1 B1;zeros(Nu,Nx) eye(Nu)];
B_ = [B1;eye(Nu)];
C_ = [C,zeros(Nx,Nu)];

%% �ס����ͦ�����
PHI_cell=cell(Np,1);
THETA_cell=cell(Np,Nc);
GAMMA_cell=cell(Np,Np);
    for i=1:1:Np
        PHI_cell{i,1}=C_*A_^i;
        for j=1:1:Nc
            if j<=i
                THETA_cell{i,j}=C_*A_^(i-j)*B_;
            else 
                THETA_cell{i,j}=zeros(Nx,Nu);
            end
        end
        for k=1:1:Np
            if k<=i
                GAMMA_cell{i,k}=C_*A_^(i-k);
            else 
                GAMMA_cell{i,k}=zeros(Nx,Nx+Nu);
            end
        end
    end
PHI=cell2mat(PHI_cell);
THETA=cell2mat(THETA_cell);
GAMMA=cell2mat(GAMMA_cell); %�ס����ͦ�����

%% �󦵵ľ�����ʽ 
phi_A_cell = cell(Np,1);
phi_B_cell = cell(Np,1);

% phi_A ����
for i=1:Np
    phi_a_cell = cell(2,Np+1);
        for k= 1:Np+1
            phi_a_cell{1,k}=zeros(Nx,Nx);
            phi_a_cell{2,k}=zeros(Nu,Nx);
        end
    phi_a_cell{1,i} = - A1 ;
    phi_a_cell{1,i+1} = eye(Nx);
    phi_a = cell2mat(phi_a_cell);       
    phi_A_cell{i} = phi_a ;
end
phi_A= cell2mat( phi_A_cell);

% phi_B ����
   for i=1:Np
        phi_B_cell{i}=[ B1;zeros(Nu,Nu)];  %% �����⣬�ǩ�B1 ���� + B1 ��
    end    
phi_B=cell2mat( phi_B_cell);

phit=phi_A * q_prediction   -   phi_B * ut; %  phi ����

%%
Q_cell = cell(Np,Np);
for i=1:1:Np
    for j=1:1:Np
        if i==j
            Q_cell{i,j}=q;
        else 
            Q_cell{i,j}=zeros(Nx,Nx);
        end 
    end
end
Q = cell2mat( Q_cell );

% Yref = kron(ones(Np,1),Yref);
Yref=[ref_position(1,:) ref_velocity(1,:)]' ; % k+1|t �Ĳο��켣
for i=1:Np-1
    Yrefnext=[ref_position(i+1,:) ref_velocity(i+1,:)]';
    Yref=[Yref ;Yrefnext];
end


%% ������
% J=(1/2)*X'HX+fX
% p=5;
% H = [THETA'*Q*THETA+R zeros(2*Nc,1);zeros(1,2*Nc) p];
H = [2*THETA'*Q*THETA + 2*R];  % x'HX ��


 
% f = [2*(PHI*statu_new)'*Q*THETA];
% f = 2*[(PHI*q_)'*Q*THETA-Yref'*Q*THETA]'; % fX ��


% f = 2*((PHI*q_)'*Q*THETA-Yref'*Q*THETA)';
f = 2 * (PHI*q_)'*Q*THETA + 2*(GAMMA*phit)'*Q*THETA-2*Yref'*Q*THETA;



%% ����ΪԼ����������
%����ʽԼ��
%     A_t=zeros(Nc,Nc);%��falcone���� P181
%     for p=1:1:Nc
%         for q=1:1:Nc
%             if q<=p
%                 A_t(p,q)=1;
%             else
%                 A_t(p,q)=0;
%             end
%         end
%     end
%     A_I=kron(A_t,eye(Nu));%��Ӧ��falcone����Լ������ľ���A,������ڿ˻�
%     Ut=kron(ones(Nc,1),u_a);%�˴��о�������Ŀ����ڿƻ�������,��ʱ����˳��
%     umin=[-1.6667;-1.6667];%ά������Ʊ����ĸ�����ͬ-1.6667
%     umax=[ 1.6667; 1.6667];
%     delta_umin=[-0.2;-0.2;];
%     delta_umax=[0.2;0.2];
%     Umin=kron(ones(Nc,1),umin);
%     Umax=kron(ones(Nc,1),umax);
%     A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
%     b_cons_cell={Umax-Ut;-Umin+Ut};
%     A_cons=cell2mat(A_cons_cell);%����ⷽ�̣�״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ
%     B_cons=cell2mat(b_cons_cell);%����ⷽ�̣�״̬������ʽԼ����ȡֵ

%%  �������� Լ��
delta_umax=[ 1 ; 2];
delta_umin=[-1 ;-2];

delta_Umax = kron(ones(Nc,1),ones(Nu,1).*delta_umax); 
delta_Umin = kron(ones(Nc,1),ones(Nu,1).*delta_umin); 

lb = [delta_Umin];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
ub = [delta_Umax];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����

%%  ������ Լ��
    A_t=zeros(Nc,Nc);% �õ�At
    for p=1:1:Nc
        for q_t=1:1:Nc
            if q_t<=p
                A_t(p,q_t)=1;
            else
                A_t(p,q_t)=0;
            end
        end
    end
    A_I = kron(A_t,eye(Nu)) ;%��Ӧ��falcone����Լ������ľ���A,������ڿ˻�
    Ut = kron(ones(Nc,1),ut) ;%�˴��о�������Ŀ����ڿƻ�������,��ʱ����˳��
    umin=[-20;  -10];%ά������Ʊ����ĸ�����ͬ Tamin Tbmin     -1.6667
    umax=[ 20;   10];
    Umin=kron(ones(Nc,1),umin.*ones(Nu,1) ) ;
    Umax=kron(ones(Nc,1),umax.*ones(Nu,1) ) ;
    
A_cons1=[A_I; -A_I];
b_cons1=[Umax-Ut; -Umin+Ut];
%   
% %% ���ٺ�ת����ٶ�Լ��
% vx_max = 6.4; vx_min = -6.4;
% gamma_max= 2;   gamma_min=-2;
% 
% vel_max = [vx_max;gamma_max];
% vel_min = [vx_min;gamma_min];
% 
% VEL_max=kron(ones(Nc,1),vel_max.*ones(Nu,1) ) ;
% VEL_min=kron(ones(Nc,1),vel_min.*ones(Nu,1) ) ;
% 
% C_beta=[0 0 0 1 0 0; 0 0 0 0 0 1]; % beta = C_beta * q + D_beta * delta_u ;
% D_beta=[0 0; 0 0];
% 
% D_I=zeros(Nc*Nu,Nc*Nu);
% 
% vel_a=[0 0 0 1 0 1]';
% 
% VELt = kron(ones(Nc,1),C_beta*vel_a) ;
% 
% VEL_max-VELt;  %betaԼ�� ���Ҳ� Ax<b
% D_I; % ���
% 
% A_cons1=[D_I;-D_I;];
% b_cons1=[VEL_max-VELt;-VEL_max+VELt;];
% 
% %% ���ֲ�ƫ��Լ��
% wheel_beta_max = deg2rad(2);
% wheel_beta_min = deg2rad(-2);
% 
% wheel_BETA_max = kron(ones(Nc,1),wheel_beta_max.*ones(Nu,1) ) ;  %%
% wheel_BETA_min = kron(ones(Nc,1),wheel_beta_min.*ones(Nu,1) ) ;
% 
% % C_beta=[0 0 0 1 0 0; 0 0 0 0 0 1]; % beta = C_beta * q + D_beta * delta_u ;
% D_beta=[0 0];
% 
% D_I=zeros(Nc,Nu);
% 
% vy=[1 2 2 ]';
% vx=[1 2 3]';
% gamma=[1 2 3]';
% 
% beta1=atan(( ( vy+l.*gamma )./( vx-d.*gamma ) )) ;    %beta1= ( vy+l*gamma )/( vx-d*gamma ) 
% beta2=atan(( ( vy+l.*gamma )./( vx+d.*gamma ) )) ;
% beta3=atan(( ( vy-l.*gamma )./( vx+d.*gamma ) )) ;
% beta4=atan(( ( vy-l.*gamma )./( vx-d.*gamma ) )) ;
% 
% vel_a=[0 0 0 1 0 1]';
% 
% % VELt = kron(ones(Nc,1),C_beta*vel_a) ;
% beta1=1;
% beta2=2;
% beta3=3;
% beta4=4;
% 
% BETA1t= kron(ones(Nc,1),[beta1;beta2;beta3;beta4]) 
% 
% A_cons1=[D_I;-D_I];        
% b_cons1=wheel_BETA_max-BETA1t;  %betaԼ�� ���Ҳ� Ax<b
% 
% 
% %% ���Ĳ�ƫ�ǵ�Լ��
% Ny1 = 1;
% 
% body_beta_max = deg2rad(5);
% body_beta_min = deg2rad(-5);
% 
% body_BETA_max =  kron(ones(Nc,1),body_beta_max.*ones(Ny1,1) ) ;
% body_BETA_min =  kron(ones(Nc,1),body_beta_min.*ones(Ny1,1) ) ;
% 
% beta=atan(vy./vx);
% 
% beta=[1 1 1];
% 
% body_BETAt=[1 1 1]';
% 
% A_cons3=[D_I;-D_I];
% b_cons3=[body_BETA_max-body_BETAt;-body_BETA_max+body_BETAt] ;  %betaԼ�� ���Ҳ� Ax<b
% 
% %% ����Լ������



%% 
% lb=[];
% ub=[];
A_cons=[A_cons1];
b_cons=[b_cons1];

Aeq=[];
beq=[];

options = optimoptions('quadprog','Algorithm','interior-point-convex', 'MaxIter',2000);
% % % options = optimset('TolFun',1E-100,'MaxIter',1E3,'MaxFunEvals',1E4,'TolX',1E-100);
%  % 'Display','iter'  ��ʾ�����ÿ�ε��������
%  % 'PlotFcns',@optimplotfval ��ÿ�ε���ʱ����Ŀ�꺯��ͼ
%  % 'MaxIter'         ���������������
%  % 'TolFun'          ����ֵ����ֹ����  
%  % 'MaxFunEvals'     ����ĺ�����ֵ��������
%  % 'TolX'            ����X��ǰ�����ֹ����
% [U,fval]=quadprog(H,f,A_cons,b_cons,Aeq,beq,lb,ub);
[U,fval]=quadprog(H,f,A_cons,b_cons,Aeq,beq,lb,ub,[],options);

u_out=U(1:2)+ut;

end