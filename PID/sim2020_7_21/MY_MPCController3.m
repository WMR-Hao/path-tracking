function [sys,x0,str,ts] = MY_MPCController3(t,x,u,flag)
%   该函数是写的第3个S函数控制器(MATLAB版本：R2011a)
%   限定于车辆运动学模型，控制量为速度和前轮偏角，使用的QP为新版本的QP解法
%   [sys,x0,str,ts] = MY_MPCController3(t,x,u,flag)
%
% is an S-function implementing the MPC controller intended for use
% with Simulink. The argument md, which is the only user supplied
% argument, contains the data structures needed by the controller. The
% input to the S-function block is a vector signal consisting of the
% measured outputs and the reference values for the controlled
% outputs. The output of the S-function block is a vector signal
% consisting of the control variables and the estimated state vector,
% potentially including estimated disturbance states.

switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs
 
 case {1,4,9} % Unused flags
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0  = [2;-2; pi/2; 0; 0; 0;];  
global U;
U=[0;0];
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.05 0];       % sample time: [period, offset]
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x;
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    global a b u_piao;
    global U;
%     U=[0;0];
    global kesi;
%     u= [2;-2; pi/2; 0; 0; 0;];
    
    
    tic 
    Nx=6;    %状态量的个数
    Nu =2;  %控制量的个数
    Np =10; %预测步长
    Nc=5;   %控制步长
    Row=10;  %松弛因子
    
    t=0.05
    fprintf('Update start, t=%6.3f\n',t)
    t_d =u(3);  %CarSim输出的为角度，角度转换为弧度
    

%    %直线路径
%     r(1)=5*t;
%     r(2)=5;
%     r(3)=0;
%     vd1=5;
%     vd2=0;
    %半径为25m的圆形轨迹,速度为5m/s
    r(1)=25*sin(0.2*t);     xr=r(1);
    r(2)=25+10-25*cos(0.2*t);   yr=r(2);
    r(3)=0.2*t; alphar=r(3);
    vd1=5;
    vd2=0.104;
    
    r(4)=5;  vxr=r(4);
    r(5)=0; vyr=r(5);
    r(6)=0.104; omegar=r(6);
    
    
    
%     %半径为25m的圆形轨迹,速度为3m/s
%     r(1)=25*sin(0.12*t);
%     r(2)=25+10-25*cos(0.12*t);
%     r(3)=0.12*t;
%     vd1=3;
%     vd2=0.104;
	%半径为25m的圆形轨迹,速度为10m/s
%      r(1)=25*sin(0.4*t);
%      r(2)=25+10-25*cos(0.4*t);
%      r(3)=0.4*t;
%      vd1=10;
%      vd2=0.104;
%     %半径为25m的圆形轨迹,速度为4m/s
%      r(1)=25*sin(0.16*t);
%      r(2)=25+10-25*cos(0.16*t);
%      r(3)=0.16*t;
%      vd1=4;
%      vd2=0.104;
    kesi=zeros(Nx+Nu,1);
    kesi(1)=u(1)-r(1);%u(1)==X(1)
    kesi(2)=u(2)-r(2);%u(2)==X(2)
    kesi(3)=u(3)-r(3); %u(3)==X(3)
    kesi(4)=u(4)-r(4);
    kesi(5)=u(5)-r(5);
    kesi(6)=u(6)-r(6);
    kesi(7)=U(1);
    kesi(8)=U(2);
    fprintf('Update start, u(1)=%4.2f\n',U(1))
    fprintf('Update start, u(2)=%4.2f\n',U(2))

    T=0.05;
    T_all=40;%临时设定，总的仿真时间，主要功能是防止计算期望轨迹越界
    % Mobile Robot Parameters
wheel_radius = 0.330/2;
d = 0.18;
l = 0.16;
m = 24;
Iz = 11.89;
c = 513.6; %轮胎侧偏刚度
    % Mobile Robot variable
    
    
%矩阵初始化   
    u_piao=zeros(Nx,Nu);
    Q=100*eye(Nx*Np,Nx*Np);    
    R=5*eye(Nu*Nc);
a=[ 1, 0, (-vxr*sin(alphar)-vyr*cos(alphar))*T, T*cos(alphar), T*sin(alphar), 0;
    0, 1, (vxr*cos(alphar)-vyr*sin(alphar))*T,  T*sin(alphar), T*cos(alphar), 0;
    0, 0, 1, 0, 0, T;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, T*(2*c*(vyr+l*omegar))/(m*vxr^2+0.00000000001), 1+(-2*c*T)/(m*vxr+0.00000000001), (-2*c*l*T)/(m*vxr+0.00000000001);
    0, 0, 0, -T*(2*c*l*(vyr+l*omegar))/(Iz*vxr^2+0.00000000001), 2*c*l*T/(Iz*vxr+0.00000000001), 1+(2*c*l*l*T)/(vxr*Iz+0.00000000001);]  %+0.0001是为了避免分母为零

b=[0 0;
   0 0;
   0 0;
   T/m T/m;
   0 0;
   -T*d/Iz T*d/Iz;]

C = eye(Nx)

    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
%     C=[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;];
    C=[C zeros(Nx,Nu)]
    PHI_cell=cell(Np,1)             %% 问题在这
    THETA_cell=cell(Np,Nc)
    for j=1:1:Np
        PHI_cell{j,1}=C*A^j
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;
            else 
                THETA_cell{j,k}=zeros(Nx,Nu);
            end
        end
    end
    PHI=cell2mat(PHI_cell)%size(PHI)=[Nx*Np Nx+Nu]
    THETA=cell2mat(THETA_cell)%size(THETA)=[Nx*Np Nu*(Nc+1)]
    
    
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell)

     error=PHI*kesi;
    f_cell=cell(1,2);
    f_cell{1,1}=2*error'*Q*THETA;
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=cell2mat(f_cell);
    
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
    Ut=kron(ones(Nc,1),U);%此处感觉论文里的克罗内科积有问题,暂时交换顺序
    umin=[-2;-2;];%维数与控制变量的个数相同
    umax=[ 2; 2];
    delta_umin=[-0.02;-0.02;];
    delta_umax=[0.02;0.002];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
    b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值
   % 状态量约束
    M=10; 
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;-M];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
    ub=[delta_Umax;M];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子
    
    %% 开始求解过程
%     options = optimset('Algorithm','active-set');
%     options = optimset('Algorithm','interior-point-convex'); 
%     [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options)
    [X,fval]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[])

    %% 计算输出
    xs=[0 1;
       2 3;
       4 5;
       6 7]
   sss=xs(3)
    
    u_piao(1)=X(1)
    u_piao(2)=X(2)
    U(1)=kesi(4)+u_piao(1)%用于存储上一个时刻的控制量
    U(2)=kesi(5)+u_piao(2)
    u_real(1)=U(1)+vd1
    u_real(2)=U(2)+vd2
    u_real
    sys= u_real
    toc
% End of mdlOutputs.