%% �໬��ģ��
% d = 0.563 / 2.0;    l = 0.324;      H = 0.242;  %�ֲ��Ϲ�����245.5
% wheel_radius = 0.165 ;      wheel_width = 0.155 ;   %�ֲ�����150
% mass_chassis = 20 ;     mass_wheel = 8 ;

function [X,Y,ALPHA,vx,dot_ALPHA]=kinematics_model_slide(X,Y,ALPHA,vx,vy,dot_ALPHA,ts)

X = ts* (vx*cos(ALPHA) - vy*sin(ALPHA)) +X;
Y = ts* (vx*sin(ALPHA) + vy*cos(ALPHA)) +Y;
ALPHA=ts*dot_ALPHA + ALPHA;

end