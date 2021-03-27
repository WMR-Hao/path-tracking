% 速度分解
function [omega_l,omega_r]=velocity_resolution(vx,omega,d,wheel_r)


B =( wheel_r/2 ).*[1 1; -1/d  1/d] ; 

v = [vx;omega];
v = B^(-1)*v;
omega_l = v(1) ;
omega_r = v(2) ;

end
