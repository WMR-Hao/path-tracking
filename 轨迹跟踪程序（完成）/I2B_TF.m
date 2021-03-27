function [vector_I]=I2B_TF(vector_B,yaw,roll,pitch)
% 坐标转换

RZ=[cos(yaw)  -sin(yaw) 0;    %绕z转yaw的旋转矩阵
    sin(yaw)  cos(yaw) 0;
    0 0 1];

RX=[1 0 0;
    0 cos(roll) -sin(roll);
    0 sin(roll) cos(roll)];

RY=[cos(pitch) 0 sin(pitch);
    0 1 0;
    -sin(pitch) 0 cos(pitch)];

% dot_Rz=[-sin(yaw) -cos(yaw) 0; 
%         cos(yaw) -sin(yaw) 0 ;
%          0 0 0 ];
% vb=RZ^(-1)*vi;

R=RX*RY*RZ;
% R=RZ*RY*RX;

% vector_I=R^(-1)*vector_B;

vector_I=R*vector_B;
% ab=Rz^(-1)*(ai- dot_Rz*vb  );
% ab=RZ^(-1)*ai;
% jb=RZ^(-1)*ji;
end