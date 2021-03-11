function [Xnext,Ynext,PSInext,vxnext,vynext,gammanext,BETA]=dynamics_model...
    (Fa,Fb,X,Y,PSI,vx,vy,gamma,dt,alpha)
%%   ����ѧģ��u=[Ta Tb] �������˶�ѧ

%% �����˲���
wheel_r = 0.11;
d = 0.197;
c_wheel = 10; %��ƫ�ն�513.6 
l = 0.1331;
m = 8.8 + 4 * 0.8;
Iz = 5.14;
g =9.81;

%% ������ѧģ��
% ��ƫ��
beta1 = atan(( ( vy+l*gamma )/( vx-d*gamma ) ));    %beta1= ( vy+l*gamma )/( vx-d*gamma ) 
beta2 = atan(( ( vy+l*gamma )/( vx+d*gamma ) ));
beta3 = atan(( ( vy-l*gamma )/( vx+d*gamma ) ));
beta4 = atan(( ( vy-l*gamma )/( vx-d*gamma ) ));

% beta1=atan2( ( vy+l*gamma ),( vx-d*gamma ) );  % ���������
% beta2=atan2( ( vy+l*gamma ),( vx+d*gamma ) );
% beta3=atan2( ( vy-l*gamma ),( vx+d*gamma ) );
% beta4=atan2( ( vy-l*gamma ),( vx-d*gamma ) );
beta_max=deg2rad(2.5);
beta_min=deg2rad(-2.5);

beta=atan(vy/vx);

BETA=[rad2deg(beta) rad2deg(beta1) rad2deg(beta2) rad2deg(beta3) rad2deg(beta4)];


% ������
F1y=-c_wheel*beta1;
F2y=-c_wheel*beta2;
F3y=-c_wheel*beta3;
F4y=-c_wheel*beta4;

% if beta1>=beta_max||beta1<=beta_min
%     if beta1>=beta_max
%         F1y=-c_wheel*beta_max;
%     else
%         F1y=-c_wheel*beta_min;
%     end 
% end
%%  
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

% F1y=-c_wheel* ( vy+l*gamma )/( vx-d*gamma ) ;
% F2y=-c_wheel* ( vy+l*gamma )/( vx+d*gamma ) ;
% F3y=-c_wheel*( ( vy-l*gamma )/( vx+d*gamma ) );
% F4y=-c_wheel*( ( vy-l*gamma )/( vx-d*gamma ) );

Fy=F1y+F2y+F3y+F4y;
% Fy=F1y+F2y-F3y-F4y

%% ������  
W=m*g/4;
W1=W;   
W2=W;
W3=W;
W4=W;

a=0.7;
b=0.36;
s=0.2;

% Fa = Ta/wheel_r - ( W1*(a*s+b)+W2*(a*s+b)+W3*(a*s+b)+W4*(a*s+b) );

f = ( W1*(a*s+b)+W2*(a*s+b)+W3*(a*s+b)+W4*(a*s+b) );
% 
% if vx>=0
%     f=f;
% else
%     f=-f;
% end
% 
% if vx>0
%     
%     Fa= Ta/wheel_r - f ;
% else
%    if vx == 0
%     Fa=0;
%    else
%        Fa= Ta/wheel_r - f ;
%    end
% end

%  Fa= Ta/wheel_r - f;  % ��ǣ����



% 
% if vx>=0 
%     Fa=Fa;
% else
%     if vx<0 && Fa <0
%     Fa=Fa;
%     else 
%         Fa=0;
%     end
% end




% Fb=Tb/wheel_r + (W1*(a*s+b)+W2*(a*s+b)-W3*(a*s+b)-W4*(a*s+b));

%%  ���棬����������ѧģ��

vxnext=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
vynext=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
% gammanext=( d*( Fb )+l*(F1y+F2y-F3y-F4y)) /Iz;
gammanext=( d*( Fb )+l*(-F1y-F2y+F3y+F4y)) /Iz;

% vynext=( (Fy-m*g*sin(alpha)*cos(PSI))/m )*dt + vy;  %-vx*gamma  ��������
% vxnext=( (Fa-m*g*sin(alpha)*sin(PSI))/m )*dt + vx;  %+vy*gamma
% gammanext=( d*( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l*(F1y+F2y-F3y-F4y)) /Iz;




%%  ���棬��������Ϊ���������Ҳ��ƫ�ǲ�ͬ��ͬ���ƫ����ͬ������ѧģ��
% F1y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
% F2y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
% F3y=-c_wheel*atan(( ( vy )/( vx+d*gamma ) ));
% F4y=-c_wheel*atan(( ( vy )/( vx-d*gamma ) ));
% Fy=F1y+F2y+F3y+F4y;
% 
% vxnext=( (Fa-m*g*sin(alpha)*sin(PSI)+m*vy*gamma)/m )*dt + vx;
% vynext=( (Fy-m*g*sin(alpha)*cos(PSI)-m*vx*gamma)/m )*dt + vy;
% gammanext=( d( Tb/wheel_r - W2*(a*s+b) - W3*(a*s+b) +W1*(a*s+b)- W4*(a*s+b) )+l(F1y+F2y-F3y-F4y)) /Iz;

%% �˶�ѧģ��
PSInext = gamma*dt + PSI; 
Xnext = ( vx*cos(PSI)-vy*sin(PSI) )*dt+X;
Ynext = ( vx*sin(PSI)+vy*cos(PSI) )*dt+Y;

end