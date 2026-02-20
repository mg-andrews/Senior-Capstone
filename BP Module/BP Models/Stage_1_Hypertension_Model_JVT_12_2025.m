clear
clc


%Initial condition
I0 = 500;

%Cardiac period Tc and systolic duration Ts
Tc = 60/72;     
Ts = (2/5)*Tc;  

%Modify parameter here to change time span
time_span = 5;

%Keep Constant
R = 1.05; C = 1.28; R1 = 0.046; L = 0.0072;

% Input Signal Functions

%Blood flow function I(t)
I = @(t) I0*sin((pi*mod(t,Tc))/Ts).^2.*(mod(t,Tc)<=Ts);

%Blood flow first derivative  I'(t)
Idot = @(t) I0*2*sin(pi*mod(t,Tc)/Ts).*cos(pi*mod(t,Tc)/Ts)*pi/Ts.*(mod(t,Tc)<=Ts);

%Blood flow second derivative I"(t)
Idotdot = @(t) I0*2*(cos(pi*mod(t,Tc)/Ts).^2 - sin(pi*mod(t,Tc)/Ts).^2)*(pi/Ts)^2.*(mod(t,Tc)<=Ts);

% 4-element Windkessel ODE - formulation online somewhere
fun = @(t,y)[y(2);
    (Idotdot(t)*(R*L*C*R1)+Idot(t)*(L*(R+R1))+I(t)*(R*R1) - (y(2)*(C*R*R1+L)+y(1)*R1))/(L*C*R)];

% Initial conditions and time span 
y0 = [80; 0];
tspan = [0 time_span*Tc];

% Solve
[T,Y] = ode45(fun,tspan,y0);



%Creating empty array for solution object
bp_data = zeros(length(Y), 2);

%Extracting solution object
for i = 1:length(Y)
    bp_data(i, 2) = Y(i);
    bp_data(i, 1) = T(i);
end 

plot(bp_data(:,1), bp_data(:,2), 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Pressure (mmHg)')
title('Arterial Blood Pressure (mmHg) vs Time (s)')
grid on

