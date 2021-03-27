function [Xnext,Ynext,PSInext,vxnext,vynext,gammanext,BETA]=dynamics_model...
    (Fdpl,Fdpr,X,Y,PSI,vx,vy,gamma,dt,alpha)
%%   动力学模型u=[Ta Tb] 后面是运动学

%% 机器人参数

d = 0.197; 
l = 0.1331;
gravity = [0 0 9.81];
wheel_r = 0.11; 
wheel_width = 0.095;
mass_chassis = 8.8;  
mass_wheel = 0.8;
g = gravity(3);
m = mass_chassis + mass_wheel * 4;
Iz = 0.14 ;  Ix = 2.8305 ;  Iy = 4.6786;
H = 0.08;  % 质心高度，未知
c_wheel = 10;

% Iz = 1; 

%% 车动力学模型
% 侧偏角
% beta1 = atan(( ( vy+l*gamma )/( vx-d*gamma ) ));    %beta1= ( vy+l*gamma )/( vx-d*gamma ) 
% beta2 = atan(( ( vy+l*gamma )/( vx+d*gamma ) ));
% beta3 = atan(( ( vy-l*gamma )/( vx+d*gamma ) ));
% beta4 = atan(( ( vy-l*gamma )/( vx-d*gamma ) ));

beta1 = ( vy+l*gamma )/( vx-d*gamma ) ;    %beta1= ( vy+l*gamma )/( vx-d*gamma ) 
beta2 = ( vy+l*gamma )/( vx+d*gamma ) ;
beta3 = ( vy-l*gamma )/( vx+d*gamma ) ;
beta4 = ( vy-l*gamma )/( vx-d*gamma ) ;


beta_max=deg2rad(7);
beta_min=deg2rad(-7);

% beta=atan(vy/vx);
beta=vy/vx ;

BETA=[rad2deg(beta) rad2deg(beta1) rad2deg(beta2) rad2deg(beta3) rad2deg(beta4)];
% 质心侧偏角 车轮侧偏角 1、2、3、4

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
vxnext=( (Fdpl+ Fdpr -m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
vynext=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
gammanext=  (( d*(-Fdpl+ Fdpr )+l*(F1y+F2y-F3y-F4y)) /Iz ) * dt +gamma;

PSInext = gamma*dt + PSI; 
Xnext = ( vx*cos(PSI)-vy*sin(PSI) )*dt+X;
Ynext = ( vx*sin(PSI)+vy*cos(PSI) )*dt+Y;

end