function [X,Y,PSI,vx,vy,gamma]=dynamics_model...
    (Ta,Tb,X,Y,PSI,vx,vy,gamma,dt,alpha)
%%   ����ѧģ��u=[Ta Tb] �������˶�ѧ

%% �����˲���
wheel_r = 0.11;
d = 0.197;
c_wheel = 300; %��ƫ�ն�513.6 
l = 0.1331;
m = 8.8 + 4 * 0.8;
Iz = 0.14;
g =9.81;

%% ������ѧģ��
% ������
F1y=-c_wheel*atan(( ( vy+l*gamma )/( vx-d*gamma ) ));
F2y=-c_wheel*atan(( ( vy+l*gamma )/( vx+d*gamma ) ));
F3y=-c_wheel*atan(( ( vy-l*gamma )/( vx+d*gamma ) ));
F4y=-c_wheel*atan(( ( vy-l*gamma )/( vx-d*gamma ) ));

Fy=F1y+F2y+F3y+F4y;

%% ������  
W=m*g/4;
W1=W;   
W2=W;
W3=W;
W4=W;

a=0.7;
b=0.36;
s=0.2;
Fa=Ta/wheel_r - (W1*(a*s+b)+W2*(a*s+b)+W3*(a*s+b)+W4*(a*s+b));

% Fb=Tb/wheel_r + (W1*(a*s+b)+W2*(a*s+b)-W3*(a*s+b)-W4*(a*s+b));

%%  ���棬����������ѧģ��
vx=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
vy=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
gamma=( d( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l(F1y+F2y-F3y-F4y)) /Iz;

%%  ���棬��������Ϊ���������Ҳ��ƫ�ǲ�ͬ��ͬ���ƫ����ͬ������ѧģ��
% F1y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
% F2y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
% F3y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
% F4y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
% Fy=F1y+F2y+F3y+F4y;
% 
% vx=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
% vy=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
% gamma=( d( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l(F1y+F2y-F3y-F4y)) /Iz;

%% �˶�ѧģ��
PSI=gamma*dt + PSI; 
X=( vx*cos(PSI)-vy*sin(PSI) )*dt+X;
Y=( vx*sin(PSI)+vy*cos(PSI) )*dt+Y;

end