% 最近修改时间：3.1
% 基于运动学模型  增量 的MPC控制器
% u=[omega_l ; omega_r]
% ref=[xr yr PSIr]

function [u_out]=my_kin_MPC_controller_v2_delta_omega(body_pos,body_vel,ref_position,dt,Np,Nc)

Q1 = [10 0 0;
      0 10 0;
      0 0 1] ;   

body_pos = double(body_pos);    ref_position = double(ref_position);
body_vel = double(body_vel); 

%% 先锋3at 底盘参数
%  参数区域     先锋3at 底盘参数

global  m Iz g alpha wheel_r l d c_wheel  Hight

omega_max = 6.4 ; %车轮最大转速
delta_omega_max = 1; 

%% 
X = body_pos(1);    Y = body_pos(2);    PSI = body_pos(3);
% Xr = ref_position(1);  Yr = ref_position(2);  PSIr = ref_position(3);

vx = body_vel(1);  % 当前时刻的速度
vy = body_vel(2);
gamma = body_vel(3);

[omega_l,omega_r]=velocity_resolution(vx,gamma,d,wheel_r);  % 速度分配计算 得到当前时刻的轮速
% [omega_lr,omega_rr]=velocity_resolution(vxr,gammar,d,wheel_radius);  % 参考速度计算


ut = [omega_l ; omega_r ] ;    %控制量及控制量个数Nu
Nu=size(ut,1);

qt = [X;   Y;   PSI];    %状态量及状态量个数Nx
Nx = size(qt,1);
   
%%   % 控制矩阵求解(将连续矩阵转换为离散矩阵，采用近似算法  A=I+T*A(t),B=T*B(t))
% 此处：delta_X=AX+BU ————X=[a-r]=[X_I-Xr;Y_I-Yr;ALPHA_I-ALPHAr]statu_e=A*statu_e+B*u_e   %离散后的模型
    % OUT=delta_X=a-r
A=[ 0, 0, -wheel_r*(omega_l+omega_r)/2*sin(PSI)-vy*cos(PSI);
    0, 0,  wheel_r*(omega_l+omega_r)/2*cos(PSI)-vy*sin(PSI);
    0, 0,  0]; 
B = [wheel_r*cos(PSI)/2  wheel_r*cos(PSI)/2;
     wheel_r*sin(PSI)/2  wheel_r*sin(PSI)/2;
          -wheel_r/(2*d)     wheel_r/(2*d);];
A1 = A * dt+eye(Nx);  % 论文中的A~、B~
B1 = B * dt;
C  = eye(Nx); 

%% 得到预测标称轨迹
q_prediction_cell = cell(Np,1);
q_prediction_cell{1,1}=[qt];

for prediction_time=1:Np
    qt_prediction = q_prediction_cell{prediction_time,1};
    Xt=qt_prediction(1);   % k|t时刻的状态量
    Yt=qt_prediction(2);
    PSIt=qt_prediction(3);
  
    vxt=ut(1);  % k|t时刻的输入量 标称轨迹上，输入恒定的输入量
    gammat=ut(2);
    
    vyt = vy ;    
    %%  运动学模型
    PSItnext = gammat * dt + PSIt;
    Xtnext = ( vxt*cos(PSIt)  - vyt*sin(PSIt) )*dt+Xt;
    Ytnext = ( vxt*sin(PSIt)  + vyt*cos(PSIt) )*dt+Yt;
    
    plot(Xtnext,Ytnext ,'o')
    hold on      
    drawnow   
    q_prediction_cell{prediction_time+1,1}=[Xtnext;Ytnext;PSItnext];   
end
q_prediction = cell2mat(q_prediction_cell);  % 在t时刻，输入量不变时，系统的状态量的理论变化 qt(k)...qt(k+Np)

% 得到新的X=A_new*X+B_new*delta_u   X=[X;u-ur]
q_=[qt;ut];   % qt当前时刻 t 的状态量   ut 前一时刻 t-1 的真实控制量
A_ = [A1 B1;zeros(Nu,Nx) eye(Nu)];
B_ = [B1;eye(Nu)];
C_ = [C,zeros(Nx,Nu)];

%%  E=A_*E+B_*U   E=X-Xr    U=u-ur
%% ψ、Θ和Γ矩阵
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
GAMMA=cell2mat(GAMMA_cell); %ψ、Θ和Γ矩阵

%% 求Φ的矩阵形式 
phi_cell = cell(Np,1);

for i=1:Np
    d_temp  = [q_prediction(Nx*i+1:Nx*(i+1)) - A1*q_prediction(Nx*i-2:Nx*i) - B1 * ut ; zeros(Nu,1)];
    phi_cell{i,1} = d_temp ;
end

phit = cell2mat(phi_cell); %  phi 矩阵

%% 权重矩阵 
Q_cell = cell(Np,Np);
for i=1:1:Np
    for j=1:1:Np
        if i==j
            Q_cell{i,j} = Q1;
        else 
            Q_cell{i,j}=zeros(Nx,Nx);
        end 
    end
end
Q = cell2mat( Q_cell );  
R = 1*eye(Nu*Nc,Nu*Nc);  
row=1; % 松弛因子 的权重

%% 
Yref=[ref_position(1,:)]' ; % k+1|t 的参考轨迹
for i=1:Np-1
    Yrefnext = [ref_position(i+1,:)]';
    Yref = [Yref ;Yrefnext];
end

%% 二次型
% J=(1/2)*X'HX+fX
H = [2*THETA'*Q*THETA + 2 * R ,  zeros(Nu*Nc,1) ;
          zeros(1,Nu*Nc),          row]   ;  % x'HX 项

N = PHI * q_ + GAMMA * phit - Yref ; 
f = [ 2*N'*Q*THETA, 0] ;    % fX 项

%% 以下为约束生成区域
%%  控制增量 约束
delta_umax = [   delta_omega_max ;   delta_omega_max];
delta_umin = [  -delta_omega_max ;  -delta_omega_max];

delta_Umax = kron(ones(Nc,1),delta_umax) ; 
delta_Umin = kron(ones(Nc,1),delta_umin) ; 

lb = [delta_Umin;0] ;
ub = [delta_Umax;10] ;

A_cons2  =  [eye(Nu*Nc) zeros(Nu*Nc,1); -eye(Nu*Nc) zeros(Nu*Nc,1)] ;
b_cons2  =  [delta_Umax; -delta_Umin] ; 

%%  控制量 约束
A_t=zeros(Nc,Nc);% 得到At
for p=1:1:Nc
    for q_t=1:1:Nc
        if q_t<=p
            A_t(p,q_t) = 1;
        else
            A_t(p,q_t) = 0;
        end
    end
end
A_I = kron(A_t,eye(Nu)) ;
Ut  = kron(ones(Nc,1),ut) ;
umin=[ - omega_max ;  - omega_max ];%维数与控制变量的个数相同 Tamin Tbmin     -1.6667
umax=[   omega_max ;    omega_max ];

Umin=kron(ones(Nc,1),umin) ;
Umax=kron(ones(Nc,1),umax) ;

A_cons1=[A_I zeros(size(A_I,1),1); -A_I zeros(size(-A_I,1),1)] ;
b_cons1=[Umax-Ut; -Umin+Ut] ;

A_cons = [A_cons1;A_cons2] ;
b_cons = [b_cons1;b_cons2] ;

% A_cons = [A_cons1] ;
% b_cons = [b_cons1] ;

lb=[];
ub=[];
Aeq=[]  ;
beq=[]  ;

% options = optimoptions('quadprog','Algorithm','interior-point-convex', 'MaxIter',2000);
%  % 'MaxIter'         允许迭代的最大次数
% [U,fval]=quadprog(H,f,A_cons,b_cons,Aeq,beq,lb,ub,[],options)

[U,fval]=quadprog(H,f,A_cons,b_cons,Aeq,beq,lb,ub)

u_out=U(1:2)+ut;

end
