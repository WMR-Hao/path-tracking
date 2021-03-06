%% ����Ŀ��Բ�������켣
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
%% ��������ȷ��
X=zeros(N,3);   % ����״̬��
X(1,:)=[15 10 pi/2];   % ��ʼ������״̬ x y yaw
U=zeros(N,2);   % ������
U(1,:)=[0 0];   % ��ʼ�������� delta velocity
vr=3.6;   % �����ٶ�
dt=0.5;   % ����ʱ��
L=2.6;   % �������
flag=1;   % ��־λ��������¼���ٵ������λ��
Kp=0.5;   % ��������
Ki=0;   % ����ʱ��
Kd=0.4;   % ΢��ʱ��
sum=0;   % ��ʼ��������
pdeg=0;   % ��ʼ��΢����
%% ���³���״̬
for i=1:1:N-1
    U(i+1,2)=U(i,2)+0.5*(vr-U(i,2));   % ���³��٣�0.5���Կ�����������ϵ��
    ld=U(i,2)*1;   % ���ݳ��ٵ���ǰ�Ӿ���
    Q=[];   % ����վ���洢Ŀ�����Ϣ
    for m=flag:1:N
        dx=Target(m,1)-X(i,1);
        dy=Target(m,2)-X(i,2);
        ds=sqrt(dx^2+dy^2);   % �����е㵽��ǰλ�õľ��룬flag��������ѡ��֮ǰ���ٹ��ĵ�
        if ds>=ld   % ѡȡ����ǰ�Ӿ���ĵ�
            temp=[ds Target(m,1) Target(m,2) m];   % ��¼���룬������Ϣ��������Ϣ
            Q=[Q;temp];
         else 
            Q=[temp];
        end
        
    end
    [min_ds, location]=min(Q(:,1));   % ȡ�복������ĵ�
    flag=Q(location,4);   % ��¼��ǰ���������Ϣ
    if flag==N   % ���������һ�����˳�
        break
    end
    deg=atan2(Q(location,3)-X(i,2),Q(location,2)-X(i,1))-X(i,3);   % ��н�a
    sum=sum+deg;   % �������
    ddeg=deg-pdeg;   % ��΢����
    U(i+1,1)=Kp*deg+Ki*sum+Kd*ddeg;   % ����ǰ��ת��
    X(i+1,1)=X(i,1)+U(i,2)*dt*cos(X(i,3));   % ���³���X����
    X(i+1,2)=X(i,2)+U(i,2)*dt*sin(X(i,3));   % ���³���Y����
    X(i+1,3)=X(i,3)+U(i,2)*dt*tan(U(i,1))/L;   % ���³�����ƫ��
    figure(1)
    plot(X(1:i,1),X(1:i,2),'r*');
    hold on
    pdeg=deg;   % ��¼�˿�ƫ����Ϊ�´�ʹ��
%     pause(0.1);
end











%%

%%%%���ֲ����Ĵ�����ѧģ�ͽ���%%%%%%
    %%��������
 L=4;%�������Ӽ�ľ���
 T=0.1;%����ʱ��
 x=0;%��ʼ��λ��x����
 y=-3;%��ʼ��λ��y����
 theta=pi/2;%��ʼ��λ��theta����
 
 x_goal=10;%�յ�λ������
 y_goal=10;%�յ�λ������
 
 x_push=[x];
 y_push=[y];
  theta_push=[ theta];
 k=10
while((x-x_goal)^2+(y-y_goal)^2 >0.01&&(x-x_goal<=0)) 
%%%%%%%%%��һ�����ø�����������һ��ֱ��%%%%%%
%%%���㵱ǰ��Ŀ��ĳ������
theta_goal=atan((y_goal-y)/(x_goal-x));
theta_error=theta-theta_goal;
u=-k*(theta_error);
 
 
 %%%%�������룬�������Ҳ�����
 vr=4+u;%����������Ҫ��ȥ����//4�����������ó��߶�죬������δ���ǵ���
 vl=4;%����������Ҫ��ȥ����
 
 
 
 %%%%process model
 
 %%%�˶�ģ��
 v=(vl+vr)/2;%�����ٶ�
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






