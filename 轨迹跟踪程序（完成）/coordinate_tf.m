clc 
clear all
close all

% 通过坐标旋转，将斜坡坐标系下的坐标，转化为平面坐标系带高度的坐标值
% x*x+y*y=4;
vr=1;
omegar=0.5;
ts=0.01;
alpha(1)=0;
x(1)=2;
y(1)=0;
z(1)=0;
yaw=0;
pitch=0;
roll=deg2rad(30);

for i=1:length([0:ts:10])
    alpha(i+1)=omegar*ts+alpha(i);
    x(i+1)=ts*vr*cos(alpha(i))+x(i);
    y(i+1)=ts*vr*sin(alpha(i))+y(i);
%     z(i+1)=tan(roll)*y(i+1);
    z(i+1)=0;
end 
 
% x=-2:0.05:2;
% y=sqrt(4-x.*x);
% figure(2)
% % plot3(x,y,z,'red')
% hold on
% figure(1)
% plot(x,y,'red')
% hold on

pos_b=[x;y;z];
for i=1:length(pos_b)
    [pos_I(:,i)]=I2B_TF(pos_b(:,i),yaw,roll,pitch);
end

figure(1)
plot(pos_I(1,:),pos_I(2,:),'blue')
hold on
figure(2)
plot3(pos_I(1,:),pos_I(2,:),pos_I(3,:),'blue')
hold on
