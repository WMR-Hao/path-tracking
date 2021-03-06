function [Xnext,Ynext,ALPHAnext,vx,omega]=kinematics_model2(omega_l,omega_r,X,Y,ALPHA,dt)
    %�˶�ѧģ��u=[omega_l omega_r]
    %% ������
    wheel_r = 0.11;
    d = 0.197;
    
    %% ���˶�ģ��
    vx=(omega_l+omega_r) * wheel_r/2;
    omega = (-omega_l+omega_r)*wheel_r/(2*d);
    
    ALPHAnext=ALPHA+omega*dt;
    Xnext=X+vx*cos(ALPHA)*dt;
    Ynext=Y+vx*sin(ALPHA)*dt;
end