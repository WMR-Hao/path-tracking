%% 参考路径
function [Xr,Yr,PSIr]=ref_path(vxr,gammar,dt,Xr,Yr,PSIr)
% vxr=1;
 vyr=0;
% omegar=0.5;
% ALPHAr=pi/4;

PSIr = dt*gammar+PSIr;
Xr = dt*(vxr*cos(PSIr) - vyr*sin(PSIr))+Xr;
Yr = dt*(vxr*sin(PSIr) + vyr*sin(PSIr))+Yr;

% %% 另一种方式 y=f（x）
% X=[0:0.01:100];
% Y=sin(X);

% vx = vX*cos(ALPHA) + vY*sin(ALPHA) ; % 速度和角速度
% gamma = (aY*vX-vY*aX)/(vX^2+vY^2) ;

end
