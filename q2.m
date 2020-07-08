%%
m = 100; %kg
g = 9.81;
A = 5*10^(-3);
V = 11*10^(-3); %m^3
Ps = 7*10^6; % Pa
cf = 9*10^3; %Ns/m
ct = 2*10^(-10);
Kq = 2*10^(-3);
dens = 850; % Kg/m^2
beta = 10^9; % Pa
Wv = 20*2*pi; % rad/sec
zeta = 0.75;
mL = 1000; %Kg
Q = 0.001667; % m^3/s
%xv = 10^(-2); %m
s = tf('s');

G1 = 1/(m*s+cf);
G2 = 1/((V/beta)*s+ct);
xv_eq = (ct*m*g)/(A*Kq*sqrt((Ps-(m*g/A))/dens));



A1=[0,1,0;0,(-cf/m),(A/m);0,((-A*beta)/V),(-beta*ct/V)*(((m*g)/(2*A*(Ps-(m*g/A))))+1)];
B=[0,0; (-1/m),0; 0,((beta*Kq)/V)*sqrt((A*Ps-m*g)/(A*dens))];
C=[1,0,0];
D=[0,0];
I = eye(3);
s = tf('s');
P = C*inv(s*I-A1)*B+D;
Pf=P([1,;]);
Pxv=P([2,;]);
Pvalve = (Wv^2)/(s^2+2*zeta*Wv*s+Wv^2);

% % plot x
figure(1)
t = out.tout;
x = out.x.signals.values;
plot(t, x,'LineWidth',2)
grid on
xlabel('time [sec]');
ylabel('elevator altitude [m]');
% hold on
% % 
% % plot Q:
% figure(2)
% t = out.tout;
% Q = out.Q.signals.values;
% plot(t, Q)
% grid on
% xlabel('time [sec]');
% ylabel('flow rate [m^3/sec]');
% 
% % plot xv:
% figure(3)
% t = out.tout;
% xv = out.xv.signals.values;
% plot(t, xv)
% grid on
% xlabel('time [sec]');
% ylabel('valve displacment [m]');

% planning controller to P2 (xv->x)


%margin(L)
grid on
% 

% check Wc found in iterations

margin(Pxv*Pvalve)
Wc = 10;
Clag = (10*s+Wc)/(10*s);
Clead = 1;

Kp = 0.7;
Ctot = Kp*Clag
margin(Ctot*Pxv*Pvalve)










