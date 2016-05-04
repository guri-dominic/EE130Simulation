clear 
close all
%% 4 Ways to initialize
% particle with default parameters
p1 = agent;

% particle with ode specified
mu = 1; k = 1;
dy_dt = @(t,y) [y(2); mu*(1-y(1)^2)*y(2)-k*y(1)];

p2 = agent(dy_dt);

% particle with ode & initial conditions specified
init = [2 3];
p3 = agent(dy_dt, init);

% particle with ode, initial conditions, and time specified
t = [0 15];
% t = [0:0.01:15];
p4 = agent(dy_dt, init, t);

%% Initialize (Required)
% the gerate the complete path with no noise
p1.initialize

%% Step
% takes one time step and returns (for now) the noise process output, just for
% that one step
p1.step

%% Run
% takes all the steps and the output is stored in a variable
p4.initialize;
p4.run;

%% Plot
p4.plotall





