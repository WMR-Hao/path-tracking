% 最近修改时间：3.1
% 基于动力学 模型的MPC控制器
% u=[Ta ; Tb]
% ref=[xr yr alphar vr omegar]

% function [u_out]=my_kin_MPC_controller(body_pos,ref_position,...
%     body_vel,ref_v,ref_gamma,body_gamma,body_vx,m,dt)
function [u_out]=my_dyn_MPC_controller(body_pos,body_vel,ref_position,ref_velocity,Ta,Tb,Ta0,Tb0,dt,alpha)
%  预测时域Np	控制时域Nc
Nc = 10 ; 
Np = 20 ;

body_pos = double(body_pos);    ref_position = double(ref_position);
body_vel = double(body_vel); 

Yref=[ref_position ref_velocity]';

u0=[Ta0; Tb0];

%% 先锋3at 底盘参数
d = 0.197;  l = 0.1331; % H = 0.2455; 
gravity = [0 0 9.81];
wheel_radius = 0.11;       wheel_width = 0.095;
mass_chassis = 8.8;            mass_wheel = 0.8;
g = gravity(3);
m = mass_chassis+mass_wheel*4;
Iz = 5.14 ;  Ix = 2.8305 ;  Iy = 4.6786;

c_wheel = 10;

%%  Nc= 25; Np=50; %预测时域Np	控制时域Nc
%    statu=[2; -2; pi/2; 0.001; 0.001; 0.001];
%    u_out=[0; 0];

X = body_pos(1);    Y = body_pos(2);    PSI = body_pos(3);
vx=body_vel(1);     vy=body_vel(2);     gamma=body_vel(3);

X0=0;  Y0=0;  PSI0=0;   vx0=0;  vy0=0;  gamma0=0;

u_a=[Ta; Tb];   %真实控制量及控制量个数Nu
Nu=size(u_a,1);

q1=[X-X0;   Y-Y0;   PSI-PSI0;  vx-vx0; vy-vy0; gamma-gamma0];    %状态量及状态量个数Nx
Nx=size(q1,1);
   
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

A1=A*dt+eye(6);
B1=B*dt;

C = eye(Nx);

q1 = C * q1; % 误差

% 得到新的X=A_new*X+B_new*delta_u   X=[X;u-ur]
A_ = [A1 B1;zeros(Nu,Nx) eye(Nu)];
B_ = [B1;eye(Nu)];
C_ = [C,zeros(Nx,Nu)];
%% 
q_ = [q1;u_a-u0];

PHI_cell=cell(Np,1);
THETA_cell=cell(Np,Nc);
    for i=1:1:Np
        PHI_cell{i,1}=C_*A_^i;
        for j=1:1:Nc
            if j<=i
                THETA_cell{i,j}=C_*A_^(i-j)*B_;
            else 
                THETA_cell{i,j}=zeros(Nx,Nu);
            end
        end
    end
PHI=cell2mat(PHI_cell);
THETA=cell2mat(THETA_cell);

q = eye(Nx);
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
R = 1*eye(Nu*Nc,Nu*Nc);
%% 二次型
% J=(1/2)*X'HX+fX
% p=5;
% H = [THETA'*Q*THETA+R zeros(2*Nc,1);zeros(1,2*Nc) p];
H = [2*THETA'*Q*THETA + 2*R];  % x'HX 项

% f = [2*(PHI*statu_new)'*Q*THETA];
% f = 2*[(PHI*q_)'*Q*THETA-Yref'*Q*THETA]'; % fX 项
Yref=kron(ones(Np,1),Yref);

f = 2*((PHI*q_)'*Q*THETA-Yref'*Q*THETA)';
%% 约束条件
% M=tril(ones(Nc));
% Im= eye(Nu);
% M=kron(M,Im);
% 
% A_cons=[];
% B_cons=[];
% delta_Umin = kron((-0.2)*ones(Nu,1),ones(Nc,1)); %速度增量约束
% delta_Umax = kron( (0.2)*ones(Nu,1),ones(Nc,1));

% lb=[delta_Umin;-5];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
% ub=[delta_Umax;5];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子



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
delta_umax=[ 1 ; 1];
delta_umin=[-1 ;-1];

delta_Umax = kron(delta_umax.*ones(Nu,1),ones(Nc,1)); 
delta_Umin = kron(delta_umin.*ones(Nu,1),ones(Nc,1)); % 
lb = [delta_Umin];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
ub = [delta_Umax];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子


%%  控制量 约束
    A_t=zeros(Nc,Nc);% 得到At
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p
                A_t(p,q)=1;
            else
                A_t(p,q)=0;
            end
        end
    end
    A_I = kron(A_t,eye(Nu)) ;%对应于falcone论文约束处理的矩阵A,求克罗内克积
    Ut = kron(ones(Nc,1),u_a) ;%此处感觉论文里的克罗内科积有问题,暂时交换顺序
    umin=[-10;  -10*0.5];%维数与控制变量的个数相同-1.6667
    umax=[ 10;   10*0.5];
    Umin=kron(ones(Nc,1),umin.*ones(Nu,1) ) ;
    Umax=kron(ones(Nc,1),umax.*ones(Nu,1) ) ;
    
%%
    A_cons = [A_I; -A_I];
    b_cons = [Umax - Ut;-Umin + Ut];
   
    

%% 车轮侧偏角beta 约束


%% 车速 约束


%% 
% lb=[];
% ub=[];
A_cons=[];
b_cons=[];

Aeq=[];
beq=[];
[U,fval]=quadprog(H,f,A_cons,b_cons,Aeq,beq,lb,ub)

% u_out=U(end-1:end)+ur;
u_out=U(1:2)+u_a;

u_out=U(1:2)+u_a;


end
