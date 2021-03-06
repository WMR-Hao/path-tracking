 % 最近修改时间：3.1
% 基于动力学 模型 的  MPC  控制器
% u=[Ta ; Tb]
% ref=[Xr Yr PSIr vxr xyr gammar]

function [u_out]=my_dyn_MPC_controller(body_pos,body_vel,ref_position,ref_velocity,Ta_previous,Tb_previous,dt,alpha)
%    body_pos,body_vel              当前t时刻的位姿和速度。
%    ref_position,ref_velocity      当前t时刻的参考位姿和参考速度。
%    Ta,Tb                          前一时刻t-1时刻的输入。
%    Ta0,Tb0                       
%    dt                             仿真步长
%    alpha                          斜坡倾角
 
%  预测时域Np	控制时域Nc
Nc = 2 ; 
Np = 20 ;

% 初始化
body_pos = double(body_pos);            body_vel = double(body_vel); 
ref_position = double(ref_position);    ref_velocity = double(ref_velocity);


X = body_pos(1);    Y = body_pos(2);    PSI = body_pos(3);  % 后边求偏导和A、B的时候用
vx=body_vel(1);     vy=body_vel(2);     gamma=body_vel(3);

%  Yref=[ref_position ref_velocity]';

ut=[Ta_previous; Tb_previous];    %前一时刻 t-1 的真实控制量及控制量个数Nu
Nu=size(ut,1);

qt=[body_pos';body_vel'];    %当前时刻 t 的状态量及状态量个数Nx
Nx=size(qt,1);

q = 1*[1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;];
R = 1*eye(Nu*Nc,Nu*Nc);

%% 先锋3at 机器人 底盘参数 
d = 0.197;  l = 0.1331; % H = 0.2455; 
gravity = [0 0 9.81];
wheel_radius = 0.11;        wheel_width = 0.095;
mass_chassis = 8.8;            mass_wheel = 0.8;
g = gravity(3);
m = mass_chassis+mass_wheel*4;
Iz = 5.14 ;  Ix = 2.8305 ;  Iy = 4.6786;

c_wheel = 10;

%% 机器人 动力学模型： 

%% 得到标称轨迹
q_prediction_cell = cell(Np,1);
% q_prediction_cell{1,1}=[X;Y;PSI;vx;vy;gamma];
q_prediction_cell{1,1}=[qt];

for prediction_time=1:Np
    qt_prediction = q_prediction_cell{prediction_time,1};
    Xt=qt_prediction(1);   % k|t时刻的状态量
    Yt=qt_prediction(2);
    PSIt=qt_prediction(3);
    vxt=qt_prediction(4);
    vyt=qt_prediction(5);
    gammat=qt_prediction(6);
        
    Fat=ut(1);  % k|t时刻的输入量 标称轨迹上，输入恒定的输入量
    Fbt=ut(2);
    
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
    
%     %% 横向力
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
    
    %%  常规，四驱，动力学模型  净牵引力情况
    vxtnext=( (Fat-m*g*sin(alpha)*sin(PSIt)+m*vyt*gammat)/m )*dt + vxt;
    vytnext=( (Fyt-m*g*sin(alpha)*cos(PSIt)-m*vxt*gammat)/m )*dt + vyt;
    gammatnext=( d*( Fbt )+l*(-F1yt-F2yt+F3yt+F4yt)) /Iz;
    
    % vynext=( (Fy-m*g*sin(alpha)*cos(PSI))/m )*dt + vy;  %-vx*gamma  向心力项
    % vxnext=( (Fa-m*g*sin(alpha)*sin(PSI))/m )*dt + vx;  %+vy*gamma
    % gammanext=( d*( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l*(F1y+F2y-F3y-F4y)) /Iz;
    
    %%  常规，四驱，简化为两驱，左右侧侧偏角不同，同侧侧偏角相同，动力学模型
    % F1y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
    % F2y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
    % F3y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
    % F4y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
    % Fy=F1y+F2y+F3y+F4y;
    %
    % vxnext=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
    % vynext=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
    % gammanext=( d( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l(F1y+F2y-F3y-F4y)) /Iz;
    
    % 运动学模型
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
    1/(wheel_radius*m) 0;
    0 0;
    0 d/(wheel_radius*Iz)];

A1=A*dt+eye(Nx);  % 论文中的A~、B~
B1=B*dt;
C = eye(Nx);

% 得到新的X=A_new*X+B_new*delta_u   X=[X;u-ur]

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
        phi_B_cell{i}=[ B1;zeros(Nu,Nu)];  %% 有问题，是﹣B1 还是 + B1 ？
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
    Yrefnext=[ref_position(i+1,:) ref_velocity(i+1,:)]';
    Yref=[Yref ;Yrefnext];
end


%% 二次型
% J=(1/2)*X'HX+fX
% p=5;
% H = [THETA'*Q*THETA+R zeros(2*Nc,1);zeros(1,2*Nc) p];
H = [2*THETA'*Q*THETA + 2*R];  % x'HX 项


 
% f = [2*(PHI*statu_new)'*Q*THETA];
% f = 2*[(PHI*q_)'*Q*THETA-Yref'*Q*THETA]'; % fX 项


% f = 2*((PHI*q_)'*Q*THETA-Yref'*Q*THETA)';
f = 2 * (PHI*q_)'*Q*THETA + 2*(GAMMA*phit)'*Q*THETA-2*Yref'*Q*THETA;



%% 以下为约束生成区域
%不等式约束
%     A_t=zeros(Nc,Nc);%见falcone论文 P181
%     for p=1:1:Nc
%         for q=1:1:Nc
%             if q<=p
%                 A_t(p,q)=1;
%             else
%                 A_t(p,q)=0;
%             end
%         end
%     end
%     A_I=kron(A_t,eye(Nu));%对应于falcone论文约束处理的矩阵A,求克罗内克积
%     Ut=kron(ones(Nc,1),u_a);%此处感觉论文里的克罗内科积有问题,暂时交换顺序
%     umin=[-1.6667;-1.6667];%维数与控制变量的个数相同-1.6667
%     umax=[ 1.6667; 1.6667];
%     delta_umin=[-0.2;-0.2;];
%     delta_umax=[0.2;0.2];
%     Umin=kron(ones(Nc,1),umin);
%     Umax=kron(ones(Nc,1),umax);
%     A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
%     b_cons_cell={Umax-Ut;-Umin+Ut};
%     A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
%     B_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值

%%  控制增量 约束
delta_umax=[ 1 ; 2];
delta_umin=[-1 ;-2];

delta_Umax = kron(ones(Nc,1),ones(Nu,1).*delta_umax); 
delta_Umin = kron(ones(Nc,1),ones(Nu,1).*delta_umin); 

lb = [delta_Umin];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
ub = [delta_Umax];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子

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
    umin=[-20;  -10];%维数与控制变量的个数相同 Tamin Tbmin     -1.6667
    umax=[ 20;   10];
    Umin=kron(ones(Nc,1),umin.*ones(Nu,1) ) ;
    Umax=kron(ones(Nc,1),umax.*ones(Nu,1) ) ;
    
A_cons1=[A_I; -A_I];
b_cons1=[Umax-Ut; -Umin+Ut];
%   
% %% 车速和转向角速度约束
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
% VEL_max-VELt;  %beta约束 的右侧 Ax<b
% D_I; % 左侧
% 
% A_cons1=[D_I;-D_I;];
% b_cons1=[VEL_max-VELt;-VEL_max+VELt;];
% 
% %% 车轮侧偏角约束
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
% b_cons1=wheel_BETA_max-BETA1t;  %beta约束 的右侧 Ax<b
% 
% 
% %% 质心侧偏角的约束
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
% b_cons3=[body_BETA_max-body_BETAt;-body_BETA_max+body_BETAt] ;  %beta约束 的右侧 Ax<b
% 
% %% 汇总约束条件



%% 
% lb=[];
% ub=[];
A_cons=[A_cons1];
b_cons=[b_cons1];

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