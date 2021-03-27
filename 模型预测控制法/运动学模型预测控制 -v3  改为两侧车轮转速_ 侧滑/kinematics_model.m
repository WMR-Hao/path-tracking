function [Xnext,Ynext,PSInext,vx,gamma]=kinematics_model(vx,gamma,X,Y,PSI,dt,beta)
    %�˶�ѧģ��u=[omega_l omega_r]
    %% ������
    wheel_r = 0.11;
    d = 0.197;
    
    %% ���˶�ģ��
   
    dot_X = vx*cos(PSI)-vx*tan(beta)*sin(PSI);
    dot_Y = vx*sin(PSI)+vx*tan(beta)*cos(PSI);
    dot_PSI = gamma;
    
    PSInext = PSI + dot_PSI*dt;
    Xnext = X + dot_X*dt;
    Ynext = Y + dot_Y*dt;
end