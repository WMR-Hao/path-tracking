function [Xnext,Ynext,PSInext,vxnext,vynext,gammanext]=dynamics_model_v3_FdpL_FdpR...
    (FdpL,FdpR,X,Y,PSI,vx,vy,gamma,dt,alpha)
%%   动力学模型u=[Ta Tb] 后面是运动学

% 改变法向载荷 与 运动学 vxnext部分

%% 机器人参数
wheel_r = 0.11;
d = 0.197;
c_wheel = 10; %侧偏刚度513.6 
l = 0.1331;
m = 8.8 + 4 * 0.8;
Iz = 0.14;
g =9.81;
H = 0.08;  % 质心高度，未知

%% 车动力学模型
% 侧偏角
beta1 = atan(( ( vy+l*gamma )/( vx-d*gamma ) ));    %beta1= ( vy+l*gamma )/( vx-d*gamma ) 
beta2 = atan(( ( vy+l*gamma )/( vx+d*gamma ) ));
beta3 = atan(( ( vy-l*gamma )/( vx+d*gamma ) ));
beta4 = atan(( ( vy-l*gamma )/( vx-d*gamma ) ));

beta_max=deg2rad(3);
beta_min=deg2rad(-3);

beta=atan(vy/vx);

BETA=[rad2deg(beta) rad2deg(beta1) rad2deg(beta2) rad2deg(beta3) rad2deg(beta4)];


% 侧向力
F1y=-c_wheel*beta1;
F2y=-c_wheel*beta2;
F3y=-c_wheel*beta3;
F4y=-c_wheel*beta4;


%%  %**************** 侧向力约束 *************************
if beta1>=beta_max
    F1y=-c_wheel*beta_max;
end
if beta1<=beta_min
    F1y=-c_wheel*beta_min;
end
%%
if beta2>=beta_max
    F2y=-c_wheel*beta_max;
end
if beta2<=beta_min
    F2y=-c_wheel*beta_min;
end
%%
if beta3>=beta_max
    F3y=-c_wheel*beta_max;
end
if beta3<=beta_min
    F3y=-c_wheel*beta_min;
end
%%
if beta4>=beta_max
    F4y=-c_wheel*beta_max;
end
if beta4<=beta_min
    F4y=-c_wheel*beta_min;
end

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

%%  常规，四驱，动力学模型
% vxnext = ((FdpL+FdpL)./(wheel_r*m)-g*sin(alpha)*sin(PSI)+vy*gamma )*dt + vx;  % 重写
% vynext = ( Fy/m - g*sin(alpha)*cos(PSI) - vx*gamma )*dt + vy ;
% gammanext = ( ( d*( -FdpL + FdpR )/wheel_r + l*(F1y+F2y-F3y-F4y))/Iz )*dt + gamma;
vxnext = ((FdpL+FdpR)./(m)-g*sin(alpha)*sin(PSI)+vy*gamma )*dt + vx;  % 重写
vynext = ( Fy/m - g*sin(alpha)*cos(PSI) - vx*gamma )*dt + vy ;
gammanext = ( ( d*( -FdpL + FdpR ) + l*(F1y+F2y-F3y-F4y))/Iz )*dt + gamma;


% vxnext=( ((FdpL+FdpL)./wheel_r-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
% vynext=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
% gammanext=( d*( (-FdpL+FdpR)/wheel_r )+l*(-F1y-F2y+F3y+F4y)) /Iz;

%% 运动学模型
PSInext = gamma*dt + PSI; 
Xnext = ( vx*cos(PSI)-vy*sin(PSI) )*dt+X;  % 改变右侧vx为vxnext
Ynext = ( vx*sin(PSI)+vy*cos(PSI) )*dt+Y;

end