function [X,Y,PSI,vx,vy,gamma]=dynamics_model_v2...
    (Ta,Tb,X,Y,PSI,vx,vy,gamma,dt,alpha)
%%   动力学模型u=[Ta Tb] 后面是运动学

% 改变法向载荷 与 运动学 vxnext部分

%% 机器人参数
wheel_r = 0.11;
d = 0.197;
c_wheel = 300; %侧偏刚度513.6 
l = 0.1331;
m = 8.8 + 4 * 0.8;
Iz = 0.14;
g =9.81;
H = 0.08;  % 质心高度，未知

%% 车动力学模型
% 侧向力
F1y=-c_wheel*atan(( ( vy+l*gamma )/( vx-d*gamma ) ));
F2y=-c_wheel*atan(( ( vy+l*gamma )/( vx+d*gamma ) ));
F3y=-c_wheel*atan(( ( vy-l*gamma )/( vx+d*gamma ) ));
F4y=-c_wheel*atan(( ( vy-l*gamma )/( vx-d*gamma ) ));

Fy=F1y+F2y+F3y+F4y;

%% 各个车轮负载
W=m*g/4;
Gx=m*g*sin(alpha)*sin(PSI);
Gy=m*g*sin(alpha)*cos(PSI);
Gz=m*g*cos(alpha);
W1=Gz/4-Gx*H/(4*l)-Gy*H/(4*d);   
W2=Gz/4-Gx*H/(4*l)+Gy*H/(4*d);
W3=Gz/4+Gx*H/(4*l)+Gy*H/(4*d);
W4=Gz/4+Gx*H/(4*l)-Gy*H/(4*d);

%% 横向力 


%% 
a=0.7;
b=0.36;
s=0.2;
Fa=Ta/wheel_r - (W1*(a*s+b)+W2*(a*s+b)+W3*(a*s+b)+W4*(a*s+b));

% Fb=Tb/wheel_r + (W1*(a*s+b)+W2*(a*s+b)-W3*(a*s+b)-W4*(a*s+b));

%%  常规，四驱，动力学模型
vxnext=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
vynext=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
% gammanext=( d*( Fb )+l*(F1y+F2y-F3y-F4y)) /Iz;
gammanext=( d*( Fb )+l*(-F1y-F2y+F3y+F4y)) /Iz;

%% 运动学模型
PSI=gammanext*dt + PSI; 
X=( vxnext*cos(PSI)-vynext*sin(PSI) )*dt+X;  % 改变右侧vx为vxnext
Y=( vxnext*sin(PSI)+vynext*cos(PSI) )*dt+Y;

end