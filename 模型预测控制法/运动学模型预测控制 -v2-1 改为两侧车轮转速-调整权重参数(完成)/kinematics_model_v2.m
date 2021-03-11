function [Xnext,Ynext,PSInext,vx,gamma]=kinematics_model_v2(omega_l,omega_r,X,Y,PSI,dt)
    %�˶�ѧģ��u=[omega_l omega_r]
    %% ������
    wheel_r = 0.11;
    d = 0.197;
    
    %% ���˶�ģ��
    vx=(omega_l+omega_r) * wheel_r/2;
    gamma = (-omega_l+omega_r)*wheel_r/(2*d);
    
    PSInext=PSI+gamma*dt;
    Xnext=X+vx*cos(PSI)*dt;
    Ynext=Y+vx*sin(PSI)*dt;
end