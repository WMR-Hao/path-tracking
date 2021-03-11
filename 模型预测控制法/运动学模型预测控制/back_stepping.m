function uout=back_stepping(X,Xr,Y,Yr,PSI,PSIr,vxr,gammar)

Xe=Xr-X;
Ye=Yr-Y;
PSIe=PSIr-PSI;

e1=cos(PSI)*(Xe)+sin(PSI)*Ye;
e2=-sin(PSI)*Xe+cos(PSI)*Ye;
e3=PSIe;

k1=1;
k2=1;
k3=1;
vx=vxr*cos(e3)+k1*e1;

gamma=gammar+k2*vxr*e2+k3*vxr*sin(e3);

uout=[vx;gamma];

