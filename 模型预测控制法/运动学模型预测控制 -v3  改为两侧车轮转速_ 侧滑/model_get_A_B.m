% 侧滑运动学新公式
clc
clear all

syms vx beta gamma  m g alpha PSI wheel_c l d Iz  X Y 


dot_X = vx*cos(PSI)-vx*tan(beta)*sin(PSI);
dot_Y = vx*sin(PSI)+vx*tan(beta)*cos(PSI);
dot_PSI = gamma;

A = jacobian([dot_X;dot_Y;dot_PSI;],[X,Y,PSI])
B = jacobian([dot_X;dot_Y;dot_PSI;],[vx,gamma])


%% 求β角的偏导
syms vt betat l gammat d beta1 beta2 beta3 beta4

beta1t = ( vt*betat+l*gammat )/( vt-d*gammat ) ;    %beta1= ( vy+l*gamma )/( vx-d*gamma ) 
beta2t = ( vt*betat+l*gammat )/( vt+d*gammat ) ;
beta3t = ( vt*betat-l*gammat )/( vt+d*gammat ) ;
beta4t = ( vt*betat-l*gammat )/( vt-d*gammat ) ;

J = jacobian([beta1;beta2;beta3;beta4],[vt,betat,gammat])
