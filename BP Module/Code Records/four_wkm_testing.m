clear all
clc   
I0= 500; % maximum flow
Tc=60/72;   % heart period
Ts=(2/5*Tc);   % time in systole
P_ss=80; % diastolic pressure
R= 1;
C= 1;
R1=0.05;    
L=0.0054;


I=@(t)I0*sin((pi*t)/Ts).^2.*(t<=Ts); %input current flow

Idot = @(t)I0*2*sin(pi*t/Ts).*cos(pi*t/Ts)*pi/Ts.*(t<=Ts);

Idotdot = @(t) I0*2*(cos(pi*t/Ts).^2 - sin(pi*t/Ts).^2)*(pi/Ts)^2.*(t<=Ts);


fun = @(t,y)[y(2);(Idotdot(t)*(R*L*C*R1)+Idot(t)*(L*(R+R1))+I(t)*(R*R1) - (y(2)*(C*R*R1+L)+y(1)*R1))/(L*C*R)];


y0 = [80 ;0];
tspan = [0 60/70];


[T,Y] = ode45(fun,tspan,y0);


plot(T,Y(:,1),'g')

disp(Y)