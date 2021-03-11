% ����޸�ʱ�䣺3.1
% �����˶�ѧģ�͵�MPC������
% u=[vx ; gamma]
% ref=[xr yr alphar vr omegar]

% function [u_out]=my_kin_MPC_controller(body_pos,ref_position,...
%     body_vel,ref_v,ref_gamma,body_gamma,body_vx,m,dt)


function [u_out]=my_kin_MPC_controller(body_pos,body_vel,ref_position,vxr,gammar,dt)
%  Ԥ��ʱ��Np	����ʱ��Nc
Nc = 10 ; 
Np = 30;
     
body_pos=double(body_pos);    ref_position=double(ref_position);
body_vel=double(body_vel); 
ref_v=vxr;
ref_gamma=gammar;
ur=[ref_v;ref_gamma];

%% �ȷ�3at ���̲���
d = 0.197;                  l = 0.1331/2; % H = 0.2455; 
gravity = [0 0 9.81];
wheel_radius = 0.11;       wheel_width = 0.095;
mass_chassis = 8.8;            mass_wheel = 0.8;
g = (mass_chassis+mass_wheel*4)*gravity(3);
I_z = 6.4414 ;  I_x = 2.8305 ;  I_y = 4.6786;

%%  Nc= 25; Np=50; %Ԥ��ʱ��Np	����ʱ��Nc
%    statu=[2; -2; pi/2; 0.001; 0.001; 0.001];
%    u_out=[0; 0];

X = body_pos(1);    Y = body_pos(2);    PSI = body_pos(3);
Xr=ref_position(1);  Yr=ref_position(2);  PSIr=ref_position(3);

vx=body_vel(1);
vy=body_vel(2);
gamma=body_vel(3);

u_a=[vx; gamma];   %������������������Nu
Nu=size(u_a,1);

statu_x=[X-Xr;   Y-Yr;   PSI-PSIr];    %״̬����״̬������Nx
Nx=size(statu_x,1);
   
%  V=R_B2I*omega2vel*u;%�˶�ѧģ��
%%
vyr= 0;
% ���ƾ������(����������ת��Ϊ��ɢ���󣬲��ý����㷨  A=I+T*A(t),B=T*B(t))
% �˴���delta_X=AX+BU ��������X=[a-r]=[X_I-Xr;Y_I-Yr;ALPHA_I-ALPHAr]statu_e=A*statu_e+B*u_e   %��ɢ���ģ��
    % OUT=delta_X=a-r
A=[ 1, 0,  (-vxr*sin(PSIr)-vyr*cos(PSIr))*dt;
    0, 1,   (vxr*cos(PSIr)-vyr*sin(PSIr))*dt;
    0, 0,  1]; 

B=[ dt*cos(PSIr) 0;
    dt*sin(PSIr) 0;
    0   dt;];  

C = eye(Nx);
error = C * statu_x; % ���

% �õ��µ�X=A_new*X+B_new*delta_u   X=[X;u-ur]
A_new = [A B;zeros(Nu,Nx) eye(Nu)];
B_new = [B;eye(Nu)];
C_new = [C,zeros(Nx,Nu)];
%% 
statu_new = [error;u_a-ur];

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
PHI=cell2mat(PHI_cell);
THETA=cell2mat(THETA_cell);

q = 100*[1 0 0 ;
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
%% ������
% J=(1/2)*X'HX+fX

H = [2*THETA'*Q*THETA + 2*R];  % x'HX ��

f = [2*(PHI*statu_new)'*Q*THETA]'; % fX ��

%% Լ������
M=tril(ones(Nc));
Im= eye(Nu);
M=kron(M,Im);

A_cons=[];
B_cons=[];
% delta_Umin = kron((-0.2)*ones(Nu,1),ones(Nc,1)); %�ٶ�����Լ��
% delta_Umax = kron( (0.2)*ones(Nu,1),ones(Nc,1));

% lb=[delta_Umin;-5];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
% ub=[delta_Umax;5];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����



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
    
Umax = kron([6.4 ;1].*ones(Nu,1),ones(Nc,1));   %  ������ Լ��
Umin = kron([-6.4 ;-1].*ones(Nu,1),ones(Nc,1)); %  ������ Լ��
lb = [Umin];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
ub = [Umax];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����
% lb=[];
% ub=[];
    

% Umax = kron(ones(Nu,1).*[6.4 ;1],ones(Nc,1));   %  ������ Լ��
% Umin = kron(ones(Nu,1),ones(Nc,1).*[-6.4 ;-1]); %  ������ Լ��
% lb = [Umin];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
% ub = [Umax];%����



   % ״̬��Լ��
%     M=10; 
%     delta_Umin=kron(ones(Nc,1),delta_umin);
%     delta_Umax=kron(ones(Nc,1),delta_umax);
%     lb=[delta_Umin;0];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
%     ub=[delta_Umax;M];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����
%     

%%
lb=[];
ub=[];

[U,fval]=quadprog(H,f,[],[],[],[],lb,ub);

% u_out=U(end-1:end)+ur;
u_out=U(1:2)+ur;


end
