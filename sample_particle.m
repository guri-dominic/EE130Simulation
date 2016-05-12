% Test different particles for the simulation:
%       Van Der Pol Oscillator (with External Force)
%       Duffing Oscillator
%       Van Der Pol Oscillator
clear

%% http://www.sharetechnote.com/html/Octave_Matlab_DifferentialEquation.html#Ode45_Vander_Pol_Oscillator_ExternalForce
mu = 1;
k = 1;
A = 2.0;
w = 10;
 
dy_dt = @(t,y) [y(2);...
                mu*(1-y(1)^2)*y(2)-k*y(1) + A*sin(w*t)];
            
odeopt = odeset ('RelTol', 0.00001, 'AbsTol', 0.00001,'InitialStep',0.5,'MaxStep',0.5);
% [t,y] = ode45(dy_dt,[0 40], [1.0 2.0],odeopt);
[t,y] = ode45(dy_dt,(0:0.01:40), [1.0 2.0],odeopt);

figure(1); clf

subplot(1,3,[1 2]);
plot(t,y(:,1),'r-',t,y(:,2),'g-')
xlabel('time'); legend('y(1)','y(2)');
grid on

subplot(1,3,3);
plot(y(:,1),y(:,2))
xlabel('y(1)'); ylabel('y(2)');
grid on

%% http://www.sharetechnote.com/html/Octave_Matlab_DifferentialEquation.html#Ode45_Duffing_Oscillator
delta = 0.06;
beta = 1.0;
w0 = 1.0;
w = 1.0;
gamma = 6.0;
phi = 0;
 
dy_dt = @(t,y) [y(2);...
                -delta*y(2)-(beta*y(1)^3 + w0^2*y(1))+gamma*cos(w*t+phi)];
            
odeopt = odeset ('RelTol', 0.00001, 'AbsTol', 0.00001,'InitialStep',0.5,'MaxStep',0.5);
[t,y] = ode45(dy_dt,[0 100], [3.0 4.1],odeopt);

figure(2); clf
subplot(1,3,[1 2]);
plot(t,y(:,1),'r-',t,y(:,2),'g-'); xlabel('time'); legend('y(1)','y(2)');
subplot(1,3,3);
plot(y(:,1),y(:,2)); xlabel('y(1)'); ylabel('y(2)');

%% http://www.sharetechnote.com/html/Octave_Matlab_DifferentialEquation.html#Ode45_Vander_Pol_Oscillator
mu = 1; k = 1;

dy_dt = @(t,y) [y(2); mu*(1-y(1)^2)*y(2)-k*y(1)];

odeopt = odeset ('RelTol', 0.00001, 'AbsTol', 0.00001,'InitialStep',0.5,'MaxStep',0.5);
init = [0.5 1.25];      % [1.0 2.0]
[t,y] = ode45(dy_dt,[0 500], init, odeopt);

figure(3); clf
subplot(1,3,[1 2]);
plot(t,y(:,1),'r-',t,y(:,2),'g-'); xlabel('time'); legend('y(1)','y(2)');
subplot(1,3,3);
plot(y(:,1),y(:,2)); xlabel('y(1)'); ylabel('y(2)');



