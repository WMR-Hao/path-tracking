function [Xnext,Ynext,PSInext,vx,gamma]=kinematics_model_v2(omega_l,omega_r,X,Y,PSI,dt,vy)
    %运动学模型u=[omega_l omega_r]
    %% 车参数
    global  m Iz g alpha wheel_r l d c_wheel  Hight
    
    %% 车运动模型
    vx    = ( omega_l + omega_r )  * wheel_r/2     ;
    gamma = ( -omega_l + omega_r ) * wheel_r/(2*d) ;
    
%     PSInext = PSI + gamma * dt;
%     Xnext = X + vx*cos(PSI)*dt ;
%     Ynext = Y + vx*sin(PSI)*dt ;
    
    PSInext = PSI + gamma * dt;
    Xnext = X + vx*cos(PSI)*dt - vy * sin(PSI)*dt;
    Ynext = Y + vx*sin(PSI)*dt + vy * cos(PSI)*dt;
end