%% �ο�·��
function [Xr,Yr,ALPHAr]=ref_path(vxr,omegar,dt,Xr,Yr,ALPHAr)
% vxr=1;
 vyr=0;
% omegar=0.5;
% ALPHAr=pi/4;

ALPHAr = dt*omegar+ALPHAr;
Xr = dt*(vxr*cos(ALPHAr) - vyr*sin(ALPHAr))+Xr;
Yr = dt*(vxr*sin(ALPHAr) + vyr*sin(ALPHAr))+Yr;

%% ��һ�ַ�ʽ y=f��x��
X=[0:0.01:100];
Y=sin(x);

vx = vX*cos(ALPHA) + vY*sin(ALPHA) ; % �ٶȺͽ��ٶ�
gamma = (aY*vX-vY*aX)/(vX^2+vY^2) ;

end
