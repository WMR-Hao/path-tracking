function [sys,x0,str,ts] = MPC_control(t,x,u,flag)

%
% The following outlines the general structure of an S-function.
%
switch flag,
  case 0,  %初始化
    [sys,x0,str,ts]=mdlInitializeSizes;
    
  case 1,  %计算导数
    sys=mdlDerivatives(t,x,u);

  case 2,  %更新离散连续量
    sys=mdlUpdate(t,x,u);

  case 3,  %计算输出
    sys=mdlOutputs(t,x,u);

  case 4,  %得到下一个仿真步长
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,  %终止
    sys=mdlTerminate(t,x,u);

  otherwise  %未知flag值
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end sfuntmpl

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
function [sys,statu0,str,ts]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0; %连续状态个数
sizes.NumDiscStates  = 6; %离散状态个数
sizes.NumOutputs     = 2; %输出量个数
sizes.NumInputs      = 6; %输入量个数
sizes.DirFeedthrough = 1; %D矩阵非空
sizes.NumSampleTimes = 1; % at least one sample time is needed
sys = simsizes(sizes);
% initialize the initial conditions
statu0  = [2;-2; pi/2; 0; 0; 0;];
global u_out;
u_out = [0;0];

% str is always an empty matrix
str = [];

ts  = [0.05 0];    % sample time: [period, offset]

% end mdlInitializeSizes

%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
function sys=mdlDerivatives(t,statu,u)

sys = [];
% end mdlDerivatives

%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
function sys=mdlUpdate(t,statu,u)

sys = statu;

% end mdlUpdate

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
function sys=mdlOutputs(t,x,statu)
  global u_out;
% statu  = [2;-2; pi/2; 0; 0; 0;];  t=0.1; u_out=[0; 0];
tic
statu
x = statu(1);   y = statu(2);  alpha = statu(3);
vx = statu(4);  vy = statu(5);    omega = statu(6);

%% 参考轨迹
vxr=1;
vyr=0;
omegar=0.5;

ref_vel=[vxr;vyr;omegar];

% dotx = vxr*cos(alphar) - vyr*sin(alphar);
% doty = vxr*sin(alphar) + vyr*sin(alphar);
% dotalpha = omegar;

xr = vxr*cos(omegar*t) - vyr*sin(omegar*t);
yr = vxr*sin(omegar*t) + vyr*sin(omegar*t);
alphar = omegar*t;

ref_pos=[xr;yr;alphar];

%% 

xr=ref_pos(1);  yr=ref_pos(2);  alphar=ref_pos(3);
vxr=ref_vel(1); vyr=ref_vel(2); omegar=ref_vel(3);

Nc= 5; Np=10; %预测时域Np	控制时域Nc                                                  运行到这

statu_error=[x-xr;   y-yr;   alpha-alphar;  vx-vxr; vy-vyr; omega-omegar]; 
Nx=size(statu_error,1);

%控制量个数Nu
Nu=size(u_out,1)
%  V=R_B2I*omega2vel*u;%运动学模型
T=0.05;
simT=20;

%% 底盘参数
d = 0.18;
l = 0.16;
m = 24;
Iz = 11.89;
c = 513.6; %轮胎侧偏刚度


% 控制矩阵求解(将连续矩阵转换为离散矩阵，采用近似算法  A=I+T*A(t),B=T*B(t))
% 此处：delta_X=AX+BU ————X=[a-r]=[X_I-Xr;Y_I-Yr;ALPHA_I-ALPHAr]statu_e=A*statu_e+B*u_e   %离散后的模型
    % OUT=delta_X=a-r
A=[ 1, 0, (-vxr*sin(alphar)-vyr*cos(alphar))*T, T*cos(alphar), T*sin(alphar), 0;
    0, 1, (vxr*cos(alphar)-vyr*sin(alphar))*T,  T*sin(alphar), T*cos(alphar), 0;
    0, 0, 1, 0, 0, T;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, T*(2*c*(vyr+l*omegar))/(m*vxr^2+0.00000000001), 1+(-2*c*T)/(m*vxr+0.00000000001), (-2*c*l*T)/(m*vxr+0.00000000001);
    0, 0, 0, -T*(2*c*l*(vyr+l*omegar))/(Iz*vxr^2+0.00000000001), 2*c*l*T/(Iz*vxr+0.00000000001), 1+(2*c*l*l*T)/(vxr*Iz+0.00000000001);]  %+0.0001是为了避免分母为零

B=[0 0;
   0 0;
   0 0;
   T/m T/m;
   0 0;
   -T*d/Iz T*d/Iz;]

C = eye(Nx)

Error = C * statu_error; % 误差

e_pos=Error(1:Nx/2); %位置误差
e_vel=Error(1+Nx/2:Nx)  %速度误差

% 得到新的X=A_new*X+B_new*delta_u   X=[X;u-ur]
% A_new = [A B;zeros(Nu,Nx) eye(Nu)]


A_new = [A B;zeros(Nu,Nx) eye(Nu)]
B_new = [B;eye(Nu)]
C_new = [C,zeros(Nx,Nu)]
%% 
ur=[0; 0];

statu_new = [Error;u_out-ur];

PHI_cell=cell(Np,1);
THETA_cell=cell(Np,Nc);
    for i=1:1:Np
        PHI_cell{i,1}=C_new*A_new^i;
        for j=1:1:Nc
            if j<=i
                THETA_cell{i,j}=C_new*A_new^(i-j)*B_new;
            else 
                THETA_cell{i,j}=zeros(Nx,Nu);
            end
        end
    end
 PHI=cell2mat(PHI_cell)
 THETA=cell2mat(THETA_cell)

q = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;]
 
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
Q = cell2mat( Q_cell )
R = 1*eye(Nu*Nc,Nu*Nc)
%% 二次型
p=5;

% ADD = THETA';
% AAA = THETA;
H = [THETA' * Q * THETA+R zeros(Nu*Nc,1);zeros(1,Nu*Nc) p]
% H = [ADD * Q * THETA+R zeros(Nu*Nc,1);zeros(1,Nu*Nc) p];
ERROR =PHI*statu_new;  %??????
G = [2*ERROR'*Q*THETA zeros(1)];
P_T=ERROR'*Q*ERROR;


%% 约束条件
M=tril(ones(Nc));
Im= eye(Nu);
M=kron(M,Im);

A_cons=[];
B_cons=[];
delta_Umin = kron((-0.2)*ones(Nu,1),ones(Nc,1)); %速度增量约束
delta_Umax = kron( (0.2)*ones(Nu,1),ones(Nc,1));

lb=[delta_Umin;-5];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
ub=[delta_Umax;5];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子


%% 以下为约束生成区域
 %不等式约束
    A_t=zeros(Nc,Nc);%见falcone论文 P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p
                A_t(p,q)=1;
            else
                A_t(p,q)=0;
            end
        end
    end
    A_I=kron(A_t,eye(Nu));%对应于falcone论文约束处理的矩阵A,求克罗内克积
    Ut=kron(ones(Nc,1),u_out);%此处感觉论文里的克罗内科积有问题,暂时交换顺序
    umin=[-10;-10];%维数与控制变量的个数相同-1.6667
    umax=[ 10; 10];
    delta_umin=[-2;2;];
    delta_umax=[2;2];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
    B_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值
% 状态量约束
M=10;
delta_Umin=kron(ones(Nc,1),delta_umin);
delta_Umax=kron(ones(Nc,1),delta_umax);
lb=[delta_Umin;-M];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
ub=[delta_Umax;M];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子

options = optimset('Algorithm','active-set');
%options = optimset('Algorithm','interior-point-convex'); 
    
[DELTA_U,fval]=quadprog(H,G,A_cons,B_cons,[],[],lb,ub,options)

u_out=DELTA_U(1:2)+u_out;
sys = u_out
toc


% end mdlOutputs

%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 0.05;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
