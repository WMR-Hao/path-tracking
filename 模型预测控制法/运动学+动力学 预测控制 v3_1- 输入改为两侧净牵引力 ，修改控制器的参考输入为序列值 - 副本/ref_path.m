%% �ο�·��
function [Xr,Yr,PSIr]=ref_path(vxr,gammar,dt,Xr,Yr,PSIr)
% vxr=1;
 vyr=0;
% omegar=0.5;
% ALPHAr=pi/4;

PSIr = dt*gammar+PSIr;
Xr = dt*(vxr*cos(PSIr) - vyr*sin(PSIr))+Xr;
Yr = dt*(vxr*sin(PSIr) + vyr*sin(PSIr))+Yr;

% %% ��һ�ַ�ʽ y=f��x��
% X=[0:0.01:100];
% Y=sin(X);

% vx = vX*cos(ALPHA) + vY*sin(ALPHA) ; % �ٶȺͽ��ٶ�
% gamma = (aY*vX-vY*aX)/(vX^2+vY^2) ;

end
