function [X,Y,PSI,vx,vy,gamma]=dynamics_model...
    (Ta,Tb,X,Y,PSI,vx,vy,gamma,dt,alpha)
%%   动力学模型u=[Ta Tb] 后面是运动学

%% 机器人参数
wheel_r = 0.11;
d = 0.197;
c_wheel = 300; %侧偏刚度513.6 
l = 0.1331;
m = 8.8 + 4 * 0.8;
Iz = 0.14;
g =9.81;

%% 车动力学模型
% 侧向力
F1y=-c_wheel*atan(( ( vy+l*gamma )/( vx-d*gamma ) ));
F2y=-c_wheel*atan(( ( vy+l*gamma )/( vx+d*gamma ) ));
F3y=-c_wheel*atan(( ( vy-l*gamma )/( vx+d*gamma ) ));
F4y=-c_wheel*atan(( ( vy-l*gamma )/( vx-d*gamma ) ));

Fy=F1y+F2y+F3y+F4y;

%% 横向力  
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

%%  常规，四驱，动力学模型
vx=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
vy=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
gamma=( d( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l(F1y+F2y-F3y-F4y)) /Iz;

%%  常规，四驱，简化为两驱，左右侧侧偏角不同，同侧侧偏角相同，动力学模型
% F1y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
% F2y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
% F3y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
% F4y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
% Fy=F1y+F2y+F3y+F4y;
% 
% vx=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
% vy=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
% gamma=( d( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l(F1y+F2y-F3y-F4y)) /Iz;

%% 运动学模型
PSI=gamma*dt + PSI; 
X=( vx*cos(PSI)-vy*sin(PSI) )*dt+X;
Y=( vx*sin(PSI)+vy*cos(PSI) )*dt+Y;

end