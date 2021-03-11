 % 最近修改时间：3.10
% 基于动力学 模型 的  MPC  控制器
% u=[FDPL ; FDPR]
% ref=[Xr Yr PSIr vxr xyr gammar]

function [u_out]=my_dyn_MPC_controller_v3(body_pos,body_vel,ref_position,ref_velocity,Fl_previous,Fr_previous,dt,alpha)
%    body_pos,body_vel              当前t时刻的位姿和速度。
%    ref_position,ref_velocity      当前t时刻的参考位姿和参考速度。
%    FDPL,TDPR                          前一时刻t-1时刻的输入。
%    FDPL0,FDPR0                       
%    dt                             仿真步长
%    alpha                          斜坡倾角
 
%  预测时域Np	控制时域Nc
Nc = 5 ; 
Np = 30 ;

% 初始化
body_pos = double(body_pos);            body_vel = double(body_vel); 
ref_position = double(ref_position);    ref_velocity = double(ref_velocity);


X = body_pos(1);    Y = body_pos(2);    PSI = body_pos(3);  % 后边求偏导和A、B的时候用
vx=body_vel(1);     vy=body_vel(2);     gamma=body_vel(3);

%  Yref=[ref_position ref_velocity]';

ut=[Fl_previous; Fr_previous];    %前一时刻 t-1 的真实控制量及控制量个数Nu
Nu=size(ut,1);

qt=[body_pos';body_vel'];    %当前时刻 t 的状态量及状态量个数Nx
Nx=size(qt,1);

q =     [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;]; % Nx*Nx
R = eye(Nu*Nc,Nu*Nc);

%% 先锋3at 机器人 底盘参数 
d = 0.197; 
l = 0.1331; % H = 0.2455; 
gravity = [0 0 9.81];
wheel_radius = 0.11;        
wheel_width = 0.095;
mass_chassis = 8.8;  
mass_wheel = 0.8;
g = gravity(3);
m = mass_chassis + mass_wheel * 4;
Iz = 0.14 ;  Ix = 2.8305 ;  Iy = 4.6786;
H = 0.08;  % 质心高度，未知
c_wheel = 10;

%% 机器人 动力学模型： 

%% 得到标称轨迹
q_prediction_cell = cell(Np,1);
% q_prediction_cell{1,1}=[X;Y;PSI;vx;vy;gamma];
q_prediction_cell{1,1} = [ qt ];

for prediction_time=1:Np
    qt_prediction = q_prediction_cell{prediction_time,1};
    Xt=qt_prediction(1);   % k|t时刻的状态量
    Yt=qt_prediction(2);
    PSIt=qt_prediction(3);
    vxt=qt_prediction(4);
    vyt=qt_prediction(5);
    gammat=qt_prediction(6);
        
    Flt=ut(1);  % k|t时刻的输入量 标称轨迹上，输入恒定的输入量
    Frt=ut(2);
    
    % 侧偏角
    beta1t = atan(( ( vyt+l*gammat )/( vxt-d*gammat ) ));    %beta1= ( vy+l*gamma )/( vx-d*gamma )
    beta2t = atan(( ( vyt+l*gammat )/( vxt+d*gammat ) ));
    beta3t = atan(( ( vyt-l*gammat )/( vxt+d*gammat ) ));
    beta4t = atan(( ( vyt-l*gammat )/( vxt-d*gammat ) ));
    
    beta_max=deg2rad(2.5);
    beta_min=deg2rad(-2.5);
    
%     betat=atan(vyt/vxt);  % 质心侧偏角
    
    % 侧向力
    F1yt=-c_wheel*beta1t;
    F2yt=-c_wheel*beta2t;
    F3yt=-c_wheel*beta3t;
    F4yt=-c_wheel*beta4t;
    %%   
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
    
W=m*g/4;
Gx=m*g*sin(alpha)*sin(PSI);
Gy=m*g*sin(alpha)*cos(PSI);
Gz=m*g*cos(alpha);
W1=Gz/4-Gx*H/(4*l)-Gy*H/(4*d);   
W2=Gz/4-Gx*H/(4*l)+Gy*H/(4*d);
W3=Gz/4+Gx*H/(4*l)+Gy*H/(4*d);
W4=Gz/4+Gx*H/(4*l)-Gy*H/(4*d);
%     
%     a=0.7;
%     b=0.36;
%     s=0.2;
%     f = ( W1*(a*s+b)+W2*(a*s+b)+W3*(a*s+b)+W4*(a*s+b) );
    % Fa = Ta/wheel_r - ( W1*(a*s+b)+W2*(a*s+b)+W3*(a*s+b)+W4*(a*s+b) );
    % Fb=Tb/wheel_r + (W1*(a*s+b)+W2*(a*s+b)-W3*(a*s+b)-W4*(a*s+b));
    
    %%  常规，四驱，动力学模型  净牵引力情况
    vxtnext=( (Flt+Frt-m*g*sin(alpha)*sin(PSIt)+ m*vyt*gammat)/m )*dt + vxt;
    vytnext=( (Fyt-m*g*sin(alpha)*cos(PSIt)- m*vxt*gammat)/m )*dt + vyt;
    gammatnext=(( d* (-Flt+Frt) +l*(F1yt+F2yt-F3yt-F4yt)) /Iz)*dt + gamma;
    
    PSItnext = gammat*dt + PSIt;
    Xtnext = ( vxt*cos(PSIt)-vyt*sin(PSIt) )*dt+Xt;
    Ytnext = ( vxt*sin(PSIt)+vyt*cos(PSIt) )*dt+Yt;
    
    q_prediction_cell{prediction_time+1,1}=[Xtnext;Ytnext;PSItnext;vxtnext;vytnext;gammatnext];
    
end
q_prediction=cell2mat(q_prediction_cell);  % 在t时刻，输入量不变时，系统的状态量的理论变化 qt(k)...qt(k+Np)


%% 车轮侧偏角的偏导 
dbeta1_dvx = (-vy-l*gamma)/((vx-d*gamma)^2);    % beta1 的 偏导数
dbeta1_dvy = 1/(vx-d*gamma);
dbeta1_dgamma = (l*vx+d*vy)/((vx-d*gamma)^2);

dbeta2_dvx = (-vy-l*gamma)/((vx+d*gamma)^2);    % beta2 的 偏导数
dbeta2_dvy = 1/(vx+d*gamma);
dbeta2_dgamma = (l*vx-d*vy)/((vx+d*gamma)^2);

dbeta3_dvx = (-vy+l*gamma)/((vx+d*gamma)^2);    % beta3 的 偏导数
dbeta3_dvy = 1/(vx+d*gamma);
dbeta3_dgamma = (-l*vx-d*vy)/((vx+d*gamma)^2);

dbeta4_dvx = (-vy+l*gamma)/((vx-d*gamma)^2);    % beta4 的 偏导数
dbeta4_dvy = 1/(vx-d*gamma);
dbeta4_dgamma = (-l*vx+d*vy)/((vx-d*gamma)^2);

sum_dbeta_dvx=dbeta1_dvx+dbeta2_dvx+dbeta3_dvx+dbeta4_dvx; % 偏导数的和
sum_dbeta_dvy=dbeta1_dvy+dbeta2_dvy+dbeta3_dvy+dbeta4_dvy;
sum_dbeta_dgamma=dbeta1_dgamma+dbeta2_dgamma+dbeta3_dgamma+dbeta4_dgamma;

diff_dbeta_dvx = -dbeta1_dvx-dbeta2_dvx+dbeta3_dvx+dbeta4_dvx;  % 偏导数的差
diff_dbeta_dvy = -dbeta1_dvy-dbeta2_dvy+dbeta3_dvy+dbeta4_dvy;
diff_dbeta_dgamma = -dbeta1_dgamma-dbeta2_dgamma+dbeta3_dgamma+dbeta4_dgamma;
%%
% 控制矩阵求解(将连续矩阵转换为离散矩阵，采用近似算法  A=I+T*A(t),B=T*B(t))
% 此处：delta_X=AX+BU ————X=[a-r]=[X_I-Xr;Y_I-Yr;ALPHA_I-ALPHAr]statu_e=A*statu_e+B*u_e   %离散后的模型
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
    1/m   1/m;
    0 0;
    -d/(Iz)   d/(Iz)]; % 分母 r 先去掉

A1=A*dt+eye(Nx);  % 论文中的A~、B~
B1=B*dt;
C = eye(Nx);

% 得到新的 X = A_new*X + B_new*delta_u   X=[X;u-ur]

q_=[qt;ut];   %qt当前时刻 t 的状态量   ut 前一时刻 t-1 的真实控制量及控制量个数Nu
A_ = [A1 B1;zeros(Nu,Nx) eye(Nu)];
B_ = [B1;eye(Nu)];
C_ = [C,zeros(Nx,Nu)];

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
phi_A_cell = cell(Np,1);
phi_B_cell = cell(Np,1);

% phi_A 矩阵
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

% phi_B 矩阵
   for i=1:Np
        phi_B_cell{i}=[ B1;zeros(Nu,Nu)];  %% 有问题，是﹣B1 还是 + B1 ？ 已解决
    end    
phi_B=cell2mat( phi_B_cell);

phit=phi_A * q_prediction   -   phi_B * ut; %  phi 矩阵

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
Yref=[ref_position(1,:) ref_velocity(1,:)]' ; % k+1|t 的参考轨迹
for i=1:Np-1
    Yrefnext = [ref_position(i+1,:) ref_velocity(i+1,:)]';
    Yref = [Yref ;Yrefnext];
end

%% 二次型
% J=(1/2)*X'HX+fX

row=1; % 松弛因子 的权重
H = [2*THETA'*Q*THETA+2*R,  zeros(Nu*Nc,1) ;
    zeros(1,Nu*Nc) ,row];  % x'HX 项
N = PHI * q_ + GAMMA * phit - Yref ; 
f = [ 2*N'*Q*THETA, 0] ;    % fX 项

%% 以下为约束生成区域
%%  控制增量 约束
p_min = -10;
p_max = 10;

delta_umax=[ 0.05 ;0.05];
delta_umin=[-0.05 ;-0.05];

delta_Umax = kron(ones(Nc,1),ones(Nu,1).*delta_umax); 
delta_Umin = kron(ones(Nc,1),ones(Nu,1).*delta_umin); 

lb = [delta_Umin;p_min];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
ub = [delta_Umax;p_max];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子

%%  控制量 约束
A_t=zeros(Nc,Nc);% 得到At
for p=1:1:Nc
    for q_t=1:1:Nc
        if q_t<=p
            A_t(p,q_t)=1;
        else
            A_t(p,q_t)=0;
        end
    end
end
A_I = kron(A_t,eye(Nu)) ;%对应于falcone论文约束处理的矩阵A,求克罗内克积
Ut = kron(ones(Nc,1),ut) ;%此处感觉论文里的克罗内科积有问题,暂时交换顺序
umin=[  -10 ;  -10];  %维数与控制变量的个数相同 Tamin Tbmin     -1.6667
umax=[   10 ;   10];

Umin=kron(ones(Nc,1),umin.*ones(Nu,1) ) ;
Umax=kron(ones(Nc,1),umax.*ones(Nu,1) ) ;

A_cons1=[A_I zeros(size(A_I,1),1); -A_I zeros(size(-A_I,1),1)] ;
b_cons1=[Umax-Ut; -Umin+Ut] ;


% A_cons=[A_cons1, zeros(size(A_cons1,1),1)];  % zeros部分是松弛变量
% b_cons=[b_cons1];

%% 车轮侧偏角约束 and 车身侧偏角
wheel_beta_max = deg2rad(5);
wheel_beta_min = deg2rad(-5);
body_beta_max = deg2rad(4);
body_beta_min = deg2rad(-4);


beta_max = [wheel_beta_max; wheel_beta_max; wheel_beta_max; wheel_beta_max; body_beta_max ];
beta_min = [wheel_beta_min; wheel_beta_min; wheel_beta_min; wheel_beta_min; body_beta_min ];
 
BETA_max = kron(ones(Np,1),beta_max ) ;  %%.*ones(Nu,1)
BETA_min = kron(ones(Np,1),beta_min ) ;
 
C_beta=[0 0 0 (-vy-l*gamma)/((vx-d*gamma)^2),  1/(vx-d*gamma), (l*vx+d*vy)/((vx-d*gamma)^2);
        0 0 0 (-vy-l*gamma)/((vx+d*gamma)^2),  1/(vx+d*gamma), (l*vx-d*vy)/((vx+d*gamma)^2);
        0 0 0 (-vy+l*gamma)/((vx+d*gamma)^2),  1/(vx+d*gamma), (-l*vx-d*vy)/((vx+d*gamma)^2);
        0 0 0 (-vy+l*gamma)/((vx-d*gamma)^2),  1/(vx-d*gamma), (-l*vx+d*vy)/((vx-d*gamma)^2);
        0 0 0 -vy/vx^2, 1/vx, 0];
  
C_BETA_cell =cell(Np,Np);
for p=1:1:Np
    for q=1:1:Np
        if q==p
            C_BETA_cell{p,q}=C_beta;
        else
            C_BETA_cell{p,q}=zeros(size(C_beta,1),Nx);
        end
    end
end
C_BETA_ = cell2mat( C_BETA_cell );

beta1_t =  atan(( ( vy+l*gamma )/( vx-d*gamma ) ));
beta2_t =  atan(( ( vy+l*gamma )/( vx+d*gamma ) ));
beta3_t =  atan(( ( vy-l*gamma )/( vx+d*gamma ) ));
beta4_t =  atan(( ( vy-l*gamma )/( vx-d*gamma ) ));
beta_t  = atan(vy/vx); 

beta_t = [beta1_t;beta2_t;beta3_t;beta4_t;beta_t] ;

BETA_t = kron(ones(Np,1),beta_t);

Cqt_cell=cell(Np,1);
for i=1:Np
   Cqt_cell{i,1}= C_beta*qt;
end
Cqt = cell2mat(Cqt_cell);

A_cons2 = [ C_BETA_*THETA ,-ones(size(C_BETA_*THETA ,1),1);
            -C_BETA_*THETA ,ones(size(-C_BETA_*THETA ,1),1)]  ;  %% ax<=b  %zeros(size(C_BETA_*THETA ,1),1)
b_cons2 = [ BETA_max - C_BETA_*PHI*q_ - C_BETA_ * GAMMA * phit - BETA_t + Cqt;
            -BETA_min + C_BETA_*PHI*q_ + C_BETA_ * GAMMA * phit + BETA_t - Cqt;];


%%  % 汇总约束条件
A_cons = [A_cons1;A_cons2] ;

b_cons = [b_cons1;b_cons2] ;


%% 
% lb=[];
% ub=[];

Aeq=[];
beq=[];

options = optimoptions('quadprog','Algorithm','interior-point-convex', 'MaxIter',2000);
% % % options = optimset('TolFun',1E-100,'MaxIter',1E3,'MaxFunEvals',1E4,'TolX',1E-100);
%  % 'Display','iter'  显示输出，每次迭代量输出
%  % 'PlotFcns',@optimplotfval 在每次迭代时绘制目标函数图
%  % 'MaxIter'         允许迭代的最大次数
%  % 'TolFun'          函数值的终止公差  
%  % 'MaxFunEvals'     允许的函数求值的最大次数
%  % 'TolX'            关于X当前点的终止公差
% [U,fval]=quadprog(H,f,A_cons,b_cons,Aeq,beq,lb,ub);

[U,fval]=quadprog(H,f,A_cons,b_cons,Aeq,beq,lb,ub,[],options);

u_out=U(1:2)+ut;

end