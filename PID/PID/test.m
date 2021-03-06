clear all


ref=20;
act=0;

ek=0;
ek1=0;
ek2=0;

ts=0.05;
t=0:ts:10;

kp=0.5;
ki=0.3;
kd=0.005;

e=[];

e(1)=ek2;
e(2)=ek1;
e(3)=ek2;

for i=1:length(t)
    val(i)=act;
    ek=ref-act;
    delta_u=kp*(ek-ek1)+ki*ek+kd*(ek-2*ek1+ek2)
    act=delta_u+act
    
    ek2=ek1;
    ek1=ek;
    
end

plot(t,val)





