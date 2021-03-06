function [vx,dot_alpha]=the_1_order_smc_controller(vxr,dot_alphar,ALPHAr,ALPHA,X,Xr,Y,Yr,vy)
%% 一阶滑模控制器
% 控制器参数
kesi1=0.025;
kesi2=0.05;

kesi1=0.6;
kesi2=0.6;

dot_vy=0;
dot_vxr=0;

e=[Xr-X;Yr-Y;ALPHAr-ALPHA];

R=[cos(ALPHA) sin(ALPHA) 0;
    -sin(ALPHA) cos(ALPHA) 0;
    0 0 1];
e1=R*e;

ex=e1(1);   ey=e1(2);   ealpha=e1(3);

s1=ex;
s2=ealpha+asin((ey-vy)/vxr);% asin(1.9)

% dA_dey= 1/( sqrt(vxr^2-(ey-vy)^2) );
% dA_dvy=-1/( sqrt(vxr^2-(ey-vy)^2) );
% dA_dvr=(-ey+vy)/(vxr*sqrt(vxr^2-(ey-vy)^2));

dA_dey= 1/( sqrt(abs(vxr^2-(ey-vy)^2)) );
dA_dvy=-1/( sqrt(abs(vxr^2-(ey-vy)^2)) );
dA_dvr=(-ey+vy)/(vxr*sqrt(abs(vxr^2-(ey-vy)^2)) );

% dot_alpha=( dot_alphar + vxr*dA_dey*sin(ealpha) - vy*dA_dey + dA_dvy*dot_vy + dA_dvr*dot_vxr + kesi2*norm(sign(s2)) )/(1+dA_dey*ex);
% vx=dot_alpha*ey+vxr*cos(ealpha)+kesi1*norm(sign(s1));

% dot_alpha=( dot_alphar + vxr*dA_dey*sin(ealpha) - vy*dA_dey + dA_dvy*dot_vy + dA_dvr*dot_vxr + kesi2*sign(s2) )/(1+dA_dey*ex);
% vx=dot_alpha*ey+vxr*cos(ealpha)+kesi1*sign(s1);

delta1=0.5;
delta2=0.5;

sat1 = s1/(abs(s1)+delta1);
sat2 = s2/(abs(s2)+delta2);

dot_alpha=( dot_alphar + vxr*dA_dey*sin(ealpha) - vy*dA_dey + dA_dvy*dot_vy + dA_dvr*dot_vxr + kesi2*sat2 )/(1+dA_dey*ex);
vx=dot_alpha*ey+vxr*cos(ealpha)+kesi1*sat1;


end