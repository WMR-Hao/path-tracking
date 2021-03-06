%% ²Î¿¼Â·¾¶
function [Xr,Yr,ALPHAr]=ref_path(vxr,omegar,dt,Xr,Yr,ALPHAr)
% vxr=1;
 vyr=0;
% omegar=0.5;
% ALPHAr=pi/4;

ALPHAr = dt*omegar+ALPHAr;
Xr = dt*(vxr*cos(ALPHAr) - vyr*sin(ALPHAr))+Xr;
Yr = dt*(vxr*sin(ALPHAr) + vyr*sin(ALPHAr))+Yr;



end
