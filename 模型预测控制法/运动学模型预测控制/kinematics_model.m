%  function [X,Y,PSI,vx,gamma]=kinematics_model(vx,gamma,X,Y,PSI,dt)
function [Xnext,Ynext,PSInext,vx,gamma]=kinematics_model(vx,gamma,X,Y,PSI,dt)
    %运动学模型u=[vx omega]
%% 侧滑用vy表示
    vy=0;
    PSInext=PSI+gamma*dt;
    Xnext=X+(vx*cos(PSI) - vy*sin(PSI))*dt;
    Ynext=Y+(vx*sin(PSI) + vy*cos(PSI))*dt;

%% 产生侧滑用xICR表示
% xICR=0.1;
% %     PSI=PSI+gamma*dt;
% %     X=X+(vx*cos(PSI) + gamma*xICR*sin(PSI))*dt;
% %     Y=Y+(vx*sin(PSI) - gamma*xICR*cos(PSI))*dt;
end