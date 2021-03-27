function [Xnext,Ynext,PSInext,vx,gamma]=kinematics_model_v2(omega_l,omega_r,X,Y,PSI,dt,beta)
    %运动学模型u=[omega_l omega_r]
    %% 车参数
    wheel_r = 0.11;
    d = 0.197;
    
    %% 车运动模型
    vx = (omega_l+omega_r) * wheel_r/2;
    gamma = (-omega_l+omega_r)*wheel_r/(2*d);
    
    dot_X = vx*cos(PSI)-vx*tan(beta)*sin(PSI);
    dot_Y = vx*sin(PSI)+vx*tan(beta)*cos(PSI);
    dot_PSI = gamma;
    
    PSInext=PSI + dot_PSI*dt;
    Xnext=X + dot_X*dt;
    Ynext=Y + dot_Y*dt;
end