function [sys,x0,str,ts] = MPC_control(t,x,u,flag)

%
% The following outlines the general structure of an S-function.
%
switch flag,
  case 0,  %��ʼ��
    [sys,x0,str,ts]=mdlInitializeSizes;
    
  case 1,  %���㵼��
    sys=mdlDerivatives(t,x,u);

  case 2,  %������ɢ������
    sys=mdlUpdate(t,x,u);

  case 3,  %�������
    sys=mdlOutputs(t,x,u);

  case 4,  %�õ���һ�����沽��
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,  %��ֹ
    sys=mdlTerminate(t,x,u);

  otherwise  %δ֪flagֵ
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

sizes.NumContStates  = 0; %����״̬����
sizes.NumDiscStates  = 6; %��ɢ״̬����
sizes.NumOutputs     = 2; %���������
sizes.NumInputs      = 6; %����������
sizes.DirFeedthrough = 1; %D����ǿ�
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

%% �ο��켣
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

Nc= 5; Np=10; %Ԥ��ʱ��Np	����ʱ��Nc                                                  ���е���

statu_error=[x-xr;   y-yr;   alpha-alphar;  vx-vxr; vy-vyr; omega-omegar]; 
Nx=size(statu_error,1);

%����������Nu
Nu=size(u_out,1)
%  V=R_B2I*omega2vel*u;%�˶�ѧģ��
T=0.05;
simT=20;

%% ���̲���
d = 0.18;
l = 0.16;
m = 24;
Iz = 11.89;
c = 513.6; %��̥��ƫ�ն�


% ���ƾ������(����������ת��Ϊ��ɢ���󣬲��ý����㷨  A=I+T*A(t),B=T*B(t))
% �˴���delta_X=AX+BU ��������X=[a-r]=[X_I-Xr;Y_I-Yr;ALPHA_I-ALPHAr]statu_e=A*statu_e+B*u_e   %��ɢ���ģ��
    % OUT=delta_X=a-r
A=[ 1, 0, (-vxr*sin(alphar)-vyr*cos(alphar))*T, T*cos(alphar), T*sin(alphar), 0;
    0, 1, (vxr*cos(alphar)-vyr*sin(alphar))*T,  T*sin(alphar), T*cos(alphar), 0;
    0, 0, 1, 0, 0, T;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, T*(2*c*(vyr+l*omegar))/(m*vxr^2+0.00000000001), 1+(-2*c*T)/(m*vxr+0.00000000001), (-2*c*l*T)/(m*vxr+0.00000000001);
    0, 0, 0, -T*(2*c*l*(vyr+l*omegar))/(Iz*vxr^2+0.00000000001), 2*c*l*T/(Iz*vxr+0.00000000001), 1+(2*c*l*l*T)/(vxr*Iz+0.00000000001);]  %+0.0001��Ϊ�˱����ĸΪ��

B=[0 0;
   0 0;
   0 0;
   T/m T/m;
   0 0;
   -T*d/Iz T*d/Iz;]

C = eye(Nx)

Error = C * statu_error; % ���

e_pos=Error(1:Nx/2); %λ�����
e_vel=Error(1+Nx/2:Nx)  %�ٶ����

% �õ��µ�X=A_new*X+B_new*delta_u   X=[X;u-ur]
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
%% ������
p=5;

% ADD = THETA';
% AAA = THETA;
H = [THETA' * Q * THETA+R zeros(Nu*Nc,1);zeros(1,Nu*Nc) p]
% H = [ADD * Q * THETA+R zeros(Nu*Nc,1);zeros(1,Nu*Nc) p];
ERROR =PHI*statu_new;  %??????
G = [2*ERROR'*Q*THETA zeros(1)];
P_T=ERROR'*Q*ERROR;


%% Լ������
M=tril(ones(Nc));
Im= eye(Nu);
M=kron(M,Im);

A_cons=[];
B_cons=[];
delta_Umin = kron((-0.2)*ones(Nu,1),ones(Nc,1)); %�ٶ�����Լ��
delta_Umax = kron( (0.2)*ones(Nu,1),ones(Nc,1));

lb=[delta_Umin;-5];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
ub=[delta_Umax;5];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����


%% ����ΪԼ����������
 %����ʽԼ��
    A_t=zeros(Nc,Nc);%��falcone���� P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p
                A_t(p,q)=1;
            else
                A_t(p,q)=0;
            end
        end
    end
    A_I=kron(A_t,eye(Nu));%��Ӧ��falcone����Լ������ľ���A,������ڿ˻�
    Ut=kron(ones(Nc,1),u_out);%�˴��о�������Ŀ����ڿƻ�������,��ʱ����˳��
    umin=[-10;-10];%ά������Ʊ����ĸ�����ͬ-1.6667
    umax=[ 10; 10];
    delta_umin=[-2;2;];
    delta_umax=[2;2];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);%����ⷽ�̣�״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ
    B_cons=cell2mat(b_cons_cell);%����ⷽ�̣�״̬������ʽԼ����ȡֵ
% ״̬��Լ��
M=10;
delta_Umin=kron(ones(Nc,1),delta_umin);
delta_Umax=kron(ones(Nc,1),delta_umax);
lb=[delta_Umin;-M];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
ub=[delta_Umax;M];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����

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
