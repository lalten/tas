%simulation einer beliebigen trajektorie

%definition durch spline:
clear all 
close all


x = [0,1,2,3,5];
y = [0,0.1,3,3,3.5];
pathxx = 0:.02:5;
pathyy = spline(x,y,pathxx);
%pathyy = linspace(1,5,length(pathxx));
plot(x,y,'o',pathxx,pathyy,'-*')
hold on 


x0=[1.5,1.0,50/180*pi];
ori = x0(1:2)+[cos(x0(3)), sin(x0(3))]*0.3; 
plot(x0(1),x0(2), 'd',[x0(1) ori(1)], [x0(2) ori(2)]);

axis equal

title('LQR for Linearized Ackermann Model')
xlabel('x')
ylabel('y')




%disp 'winkel nähester abschnitt'
%xn(3)/pi*180

%plot(xn(1),xn(2), 'd', 'MarkerSize',5,'markerfacecolor','b')



%%%%%%%%%%%%%%%% Regler
%systemmatrix

a22=0.001;

v= 0.2;

A = [a22/v 0   0;
     -1    0   0;
      0    v   0];

b2 = 22;

B= [b2; 0 ; 0];

%eingangsmatrix

%realer lenkwinkel
delta_l = 30/180*pi;

%simulationsschritte
nsim = 7000;

xposdot = zeros(nsim,1);
yposdot = zeros(nsim,1);
xpos = zeros(nsim,1);
ypos = zeros(nsim,1);

psi = zeros(nsim,1);
psidot = zeros(nsim,1);
psidotdot = zeros(nsim,1);

thetad = zeros(nsim,1);
thetaddot = zeros(nsim,1);

yd = zeros(nsim,1);

x = zeros(3,nsim);
xdot = zeros(3,nsim);


dt= 0.001;

Q=[0  0 0;
    0  100 0;
    0 0  800];

R=30;
 
 
[K,S,e] = lqr(A,B,Q,R,zeros(3,1));
K
u = zeros(1,nsim);
xd = zeros(3,nsim);

xpos(1)=x0(1);
ypos(1)=x0(2);
psi(1) = x0(3);

x(2,1)=x0(3);


for t = 1:nsim-1

    xposdot(t) = v*cos(psi(t));
    yposdot(t) = v*sin(psi(t));

    xpos(t+1)= xpos(t)+ xposdot(t)*dt;
    ypos(t+1)= ypos(t)+ yposdot(t)*dt;

    %u=delta_l;  %% konstanter lenkwinkel
    %geregelter lenkwinkel:

    %berechne nähesten punkt:

    l= zeros(1,length(pathxx));

    for i=1:length(pathxx)
        l(i)= sqrt( (pathxx(i)-xpos(t))^2 + (pathyy(i)-ypos(t))^2);
    end

    [~, min_index] = min(l);
    xn(1:2) = [pathxx(min_index),pathyy(min_index)];
    xnn(1:2) = [pathxx(min_index+1),pathyy(min_index+1)];
    xn(3) = atan2(xnn(2)-xn(2),xnn(1)-xn(1));
    xn(3)
     
    %lotf = [cos(xn(3)+90), sin(xn(3)+90)];
    lotf = [cos(xn(3)+pi/4), sin(xn(3)+pi/4)];
    
    verb_vek = xn(1:2)- [xpos(t),ypos(t)];  %verbinungsvektor
    lateral_d = lotf*verb_vek';
    
    if t==1
        xd(:,t) = [psidot(t),xn(3) - psi(t),lateral_d];
    else
        xd(:,t) = [psidot(t-1),xn(3) - psi(t),lateral_d];
    end
    
    if mod(t-1,100)==0
        plot([xpos(t),xn(1)],[ypos(t),xn(2)]);
    end
    
    
    e(:,t) = xd(:,t);
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

 
plot(xpos,ypos)



figure
subplot(2,1,1)
plot(1:nsim-1,e(1,:),1:nsim-1,e(3,:))
legend('epsidot','eyd')

subplot(2,1,2)
plot(1:nsim-1,e(2,:)*180/pi)
legend('etheta')


figure
plot(u/pi*180)
title u


    












