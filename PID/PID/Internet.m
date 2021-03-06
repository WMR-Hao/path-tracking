%% 生成目标圆形期望轨迹
N=200;
Target=zeros(N,2);
for i=1:1:200
    Target(i,1)=20*cos(2*pi/200*i);
    Target(i,2)=20*sin(2*pi/200*i);
%     Target(i,1)=i;
%     Target(i,2)=5;
end
figure(1)
plot(Target(:,1),Target(:,2));
hold on
axis equal
%% 车辆参数确定
X=zeros(N,3);   % 车辆状态量
X(1,:)=[15 10 pi/2];   % 初始化车辆状态 x y yaw
U=zeros(N,2);   % 控制量
U(1,:)=[0 0];   % 初始化控制量 delta velocity
vr=3.6;   % 期望速度
dt=0.5;   % 采样时间
L=2.6;   % 车辆轴距
flag=1;   % 标志位，用来记录跟踪点的序列位置
Kp=0.5;   % 比例增益
Ki=0;   % 积分时间
Kd=0.4;   % 微分时间
sum=0;   % 初始化积分项
pdeg=0;   % 初始化微分项
%% 更新车辆状态
for i=1:1:N-1
    U(i+1,2)=U(i,2)+0.5*(vr-U(i,2));   % 更新车速，0.5可以看做比例调节系数
    ld=U(i,2)*1;   % 根据车速调节前视距离
    Q=[];   % 定义空矩阵存储目标点信息
    for m=flag:1:N
        dx=Target(m,1)-X(i,1);
        dy=Target(m,2)-X(i,2);
        ds=sqrt(dx^2+dy^2);   % 求所有点到当前位置的距离，flag用来避免选定之前跟踪过的点
        if ds>=ld   % 选取满足前视距离的点
            temp=[ds Target(m,1) Target(m,2) m];   % 记录距离，坐标信息，序列信息
            Q=[Q;temp];
         else 
            Q=[temp];
        end
        
    end
    [min_ds, location]=min(Q(:,1));   % 取离车辆最近的点
    flag=Q(location,4);   % 记录当前点的序列信息
    if flag==N   % 跟踪完最后一个点退出
        break
    end
    deg=atan2(Q(location,3)-X(i,2),Q(location,2)-X(i,1))-X(i,3);   % 求夹角a
    sum=sum+deg;   % 求积分项
    ddeg=deg-pdeg;   % 求微分项
    U(i+1,1)=Kp*deg+Ki*sum+Kd*ddeg;   % 更新前轮转角
    X(i+1,1)=X(i,1)+U(i,2)*dt*cos(X(i,3));   % 更新车辆X坐标
    X(i+1,2)=X(i,2)+U(i,2)*dt*sin(X(i,3));   % 更新车辆Y坐标
    X(i+1,3)=X(i,3)+U(i,2)*dt*tan(U(i,1))/L;   % 更新车辆航偏角
    figure(1)
    plot(X(1:i,1),X(1:i,2),'r*');
    hold on
    pdeg=deg;   % 记录此刻偏差作为下次使用
%     pause(0.1);
end











%%

%%%%两轮差速履带车数学模型建立%%%%%%
    %%参数设置
 L=4;%两个轮子间的距离
 T=0.1;%采样时间
 x=0;%初始化位置x坐标
 y=-3;%初始化位置y坐标
 theta=pi/2;%初始化位置theta坐标
 
 x_goal=10;%终点位置坐标
 y_goal=10;%终点位置坐标
 
 x_push=[x];
 y_push=[y];
  theta_push=[ theta];
 k=10
while((x-x_goal)^2+(y-y_goal)^2 >0.01&&(x-x_goal<=0)) 
%%%%%%%%%这一段设置跟踪器，跟踪一段直线%%%%%%
%%%计算当前与目标的朝向误差
theta_goal=atan((y_goal-y)/(x_goal-x));
theta_error=theta-theta_goal;
u=-k*(theta_error);
 
 
 %%%%控制输入，左电机和右侧电机。
 vr=4+u;%控制输入需要你去更改//4代表着你想让车走多快，我这里未考虑到。
 vl=4;%控制输入需要你去更改
 
 
 
 %%%%process model
 
 %%%运动模型
 v=(vl+vr)/2;%中心速度
 w=(vr-vl)/L;
 x=x+v*cos(theta)*T;
 y=y+v*sin(theta)*T;
 theta=theta+w*T;
 x_push=[x_push;x];
 y_push=[y_push;y];
 theta_push=[theta_push;theta];
end

plot(x_push,y_push);
axis([-5  12 -5 12])






