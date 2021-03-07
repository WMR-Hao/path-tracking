% 最近修改时间：3.1
% 基于运动学模型的MPC控制器
% u=[omega_l ; omega_r]
% ref=[xr yr PSIr vr gammar]

function [u_out]=my_kin_MPC_controller_v2(body_pos,body_vel,ref_position,vxr,vyr,gammar,dt)
%  预测时域Np	控制时域Nc
Nc = 5 ; 
Np = 20 ;
     
body_pos = double(body_pos);    ref_position = double(ref_position);
body_vel = double(body_vel); 

%% 先锋3at 底盘参数
d = 0.197;                  l = 0.1331/2; % H = 0.2455; 
gravity = [0 0 9.81] ;
wheel_radius = 0.11 ;       wheel_width = 0.095;
mass_chassis = 8.8 ;            mass_wheel = 0.8;
g = (mass_chassis+mass_wheel*4)*gravity(3);
I_z = 6.4414 ;  I_x = 2.8305 ;  I_y = 4.6786;
omega_max=6.4; %车轮最大转速

X = body_pos(1);    Y = body_pos(2);    PSI = body_pos(3);
Xr=ref_position(1);  Yr=ref_position(2);  PSIr=ref_position(3);

vx=body_vel(1);
vy=body_vel(2);
gamma=body_vel(3);

[omega_l,omega_r]=velocity_resolution(vx,gamma,d,wheel_radius);  % 速度分配计算
[omega_lr,omega_rr]=velocity_resolution(vxr,gammar,d,wheel_radius);  % 参考速度计算

ur=[omega_lr;omega_rr];

u_a=[omega_l; omega_r];   %控制量及控制量个数Nu
Nu=size(u_a,1);

e_q=[X-Xr;   Y-Yr;   PSI-PSIr];    %状态量及状态量个数Nx
Nx=size(e_q,1);
   
%%   % 控制矩阵求解(将连续矩阵转换为离散矩阵，采用近似算法  A=I+T*A(t),B=T*B(t))
% 此处：delta_X=AX+BU ————X=[a-r]=[X_I-Xr;Y_I-Yr;ALPHA_I-ALPHAr]statu_e=A*statu_e+B*u_e   %离散后的模型
    % OUT=delta_X=a-r
A=[ 1, 0, -wheel_radius*dt*(omega_lr+omega_rr)/2*sin(PSIr)-dt*vyr*cos(PSIr);
    0, 1,  wheel_radius*dt*(omega_lr+omega_rr)/2*cos(PSIr)-dt*vyr*sin(PSIr);
    0, 0,  1]; 
B = [dt*wheel_radius*cos(PSIr)/2  dt*wheel_radius*cos(PSIr)/2;
     dt*wheel_radius*sin(PSIr)/2  dt*wheel_radius*sin(PSIr)/2;
     -dt*wheel_radius/(2*d)     dt*wheel_radius/(2*d);];

%  E=A_*E+B_*U   E=X-Xr    U=u-ur

%% 求论文中的A_和B_
A_cell=cell(Np,1);
B_cell=cell(Np,Nc);
    for i=1:1:Np
        A_cell{i,1}=A^i;
        for j=1:1:Nc
            if j<=i
                B_cell{i,j}=A^(i-j)*B;
            else 
                B_cell{i,j}=zeros(Nx,Nu);
            end
        end
    end
A_=cell2mat(A_cell);
B_=cell2mat(B_cell);

q = 50*[1 0 0 ;
        0 1 0 ;
        0 0 1];
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
%   % J = (1/2)*X'HX+fX
%   %  p=5;

H = [2*B_'*Q*B_ + 2*R];  % x'HX 项
f = [2*(A_*e_q)'*Q*B_]'; % fX 项

%% 约束条件
M=tril(ones(Nc));
Im= eye(Nu);
M=kron(M,Im);

A_cons=[];
B_cons=[];
    
Umax = kron([omega_max ].*ones(Nu,1),ones(Nc,1));   %  控制量 约束
Umin = kron([-omega_max ].*ones(Nu,1),ones(Nc,1)); %  控制量 约束

Ur = kron( ur .* ones(Nu,1),ones(Nc,1)); % 参考轨迹的输入速度

lb = Umin-Ur;  %（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
ub = Umax-Ur;  %（求解方程）状态量上界，包含控制时域内控制增量和松弛因子
    
[U,fval]=quadprog(H,f,[],[],[],[],lb,ub);

% u_out=U(end-1:end)+ur;
u_out=U(1:2)+ur;


end
