function [Xnext,Ynext,PSInext,vxnext,vynext,gammanext]=dynamics_model_v3_FdpL_FdpR...
    (FdpL,FdpR,X,Y,PSI,vx,vy,gamma,dt,alpha)
%%   ����ѧģ��u=[Ta Tb] �������˶�ѧ

% �ı䷨���غ� �� �˶�ѧ vxnext����

%% �����˲���
wheel_r = 0.11;
d = 0.197;
c_wheel = 10; %��ƫ�ն�513.6 
l = 0.1331;
m = 8.8 + 4 * 0.8;
Iz = 0.14;
g =9.81;
H = 0.08;  % ���ĸ߶ȣ�δ֪

%% ������ѧģ��
% ��ƫ��
beta1 = atan(( ( vy+l*gamma )/( vx-d*gamma ) ));    %beta1= ( vy+l*gamma )/( vx-d*gamma ) 
beta2 = atan(( ( vy+l*gamma )/( vx+d*gamma ) ));
beta3 = atan(( ( vy-l*gamma )/( vx+d*gamma ) ));
beta4 = atan(( ( vy-l*gamma )/( vx-d*gamma ) ));

beta_max=deg2rad(3);
beta_min=deg2rad(-3);

beta=atan(vy/vx);

BETA=[rad2deg(beta) rad2deg(beta1) rad2deg(beta2) rad2deg(beta3) rad2deg(beta4)];


% ������
F1y=-c_wheel*beta1;
F2y=-c_wheel*beta2;
F3y=-c_wheel*beta3;
F4y=-c_wheel*beta4;


%%  %**************** ������Լ�� *************************
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

%% �������ָ���
W=m*g/4;
Gx=m*g*sin(alpha)*sin(PSI);
Gy=m*g*sin(alpha)*cos(PSI);
Gz=m*g*cos(alpha);
W1=Gz/4-Gx*H/(4*l)-Gy*H/(4*d);   
W2=Gz/4-Gx*H/(4*l)+Gy*H/(4*d);
W3=Gz/4+Gx*H/(4*l)+Gy*H/(4*d);
W4=Gz/4+Gx*H/(4*l)-Gy*H/(4*d);

%%  ���棬����������ѧģ��
% vxnext = ((FdpL+FdpL)./(wheel_r*m)-g*sin(alpha)*sin(PSI)+vy*gamma )*dt + vx;  % ��д
% vynext = ( Fy/m - g*sin(alpha)*cos(PSI) - vx*gamma )*dt + vy ;
% gammanext = ( ( d*( -FdpL + FdpR )/wheel_r + l*(F1y+F2y-F3y-F4y))/Iz )*dt + gamma;
vxnext = ((FdpL+FdpR)./(m)-g*sin(alpha)*sin(PSI)+vy*gamma )*dt + vx;  % ��д
vynext = ( Fy/m - g*sin(alpha)*cos(PSI) - vx*gamma )*dt + vy ;
gammanext = ( ( d*( -FdpL + FdpR ) + l*(F1y+F2y-F3y-F4y))/Iz )*dt + gamma;


% vxnext=( ((FdpL+FdpL)./wheel_r-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
% vynext=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
% gammanext=( d*( (-FdpL+FdpR)/wheel_r )+l*(-F1y-F2y+F3y+F4y)) /Iz;

%% �˶�ѧģ��
PSInext = gamma*dt + PSI; 
Xnext = ( vx*cos(PSI)-vy*sin(PSI) )*dt+X;  % �ı��Ҳ�vxΪvxnext
Ynext = ( vx*sin(PSI)+vy*cos(PSI) )*dt+Y;

end