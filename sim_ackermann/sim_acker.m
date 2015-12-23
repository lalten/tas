%zustandraumodell für ackermann fahrzeug
clc
clear all
close all


%%%%%%%%%%%%%%%% Regler
%systemmatrix

a22=0.001;

v= 1;

A = [a22/v 0   0;
     -1    0   0;
      0    v   0];

b2 = 22;

B= [b2; 0 ; 0];

%eingangsmatrix

%realer lenkwinkel
delta_l = 30/180*pi;

%simulationsschritte
nsim = 1000;

xposdot = zeros(nsim,1);
yposdot = zeros(nsim,1);
xpos = zeros(nsim,1);
ypos = zeros(nsim,1);

psi = zeros(nsim,1);
psidot = zeros(nsim,1);
psidotdot = zeros(nsim,1);

%beta = zeros(nsim,1);
%betadot = zeros(nsim,1);

thetad = zeros(nsim,1);
thetaddot = zeros(nsim,1);

yd = zeros(nsim,1);

x = zeros(3,nsim);
xdot = zeros(3,nsim);


dt= 0.001;

Q=[0  0 0;
    0  30 0;
    0 0  1000];

R=10;
 
 
[K,S,e] = lqr(A,B,Q,R,zeros(3,1));
K
u = zeros(1,nsim);

%sollposition, x, y, orientierung
xd=zeros(3,nsim);

ha =0.05;
la = 1; %0.50;
% for n= 1:nsim
%     xs(2,n)=(-cos(pi*n/nsim)+1)*ha;
%     xs(1,n)=la*n/nsim;
%     xs(3,n)=atan(sin(pi*n/nsim)*pi/nsim*ha)*180/pi;
% end

xd(1,:)=zeros(1,nsim);
xd(2,:)=zeros(1,nsim);
xd(3,:)=ones(1,nsim)*ha;

zeros(3,nsim);

 for t = 1:nsim-1
     
     xposdot(t) = v*cos(psi(t));
     yposdot(t) = v*sin(psi(t));
     
     xpos(t+1)= xpos(t)+ xposdot(t)*dt;
     ypos(t+1)= ypos(t)+ yposdot(t)*dt;

     %u=delta_l;  %% konstanter lenkwinkel
     %geregelter lenkwinkel:
     e(:,t)= x(:,t) + xd(:,t);
     u(t) = -K*e(:,t);
     
     xdot(:,t) = A*x(:,t) + B*u(t);
     %xdot(:,t) = A*x(:,t) + B*u;
     x(:,t+1) = x(:,t) + xdot(:,t) * dt;
     
     psidotdot(t) = xdot(1,t);
     psidot(t) = x(1,t);
     thetad(t) = x(2,t);
     yd(t) = x(3,t);
     
     psi(t+1) = psi(t) + psidot(t)*dt;
     
     
 end
 
  
 
%referenz kreis
r1=0.6; %
for n= 1:nsim
    phi= 45*pi/180*n/nsim;
    xr(n)= r1*sin(phi);
    yr(n)= r1-cos(phi)*r1;
end

%soll-trajekorie:
r1=0.6; %


    

figure

plot(xpos,ypos, [1:nsim]*la/nsim,xd(3,:));
title('position')
axis equal


figure

subplot(3,1,1);
plot(psidot);
title('psidot')


subplot(3,1,2);
plot(thetad);
title('thetad')

subplot(3,1,3);
plot(yd);
title('yd')

figure
plot(u/pi*180)
title u

figure
plot(1:nsim-1,e(1,:),1:nsim-1,e(2,:),1:nsim-1,e(3,:))
legend('epsi','etheta','eyd')

% figure
% subplot(2,1,1);
% plot(yd);
% title('yd')
% 
% subplot(2,1,2);
% plot(thetad);
% title('thetad')



     