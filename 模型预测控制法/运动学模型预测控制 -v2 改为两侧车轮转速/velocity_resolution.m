% 速度分解
function [omega_l,omega_r]=velocity_resolution(vel,omega,d,wheel_r)

B=(1/2).*[ 1   1;
    -1/d 1/d];
v=[vel;omega];
v=B^(-1)*v;

omega_l=v(1)/wheel_r;
omega_r=v(2)/wheel_r;
end
