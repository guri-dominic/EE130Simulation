



a = agent;
a.t = [0 30];

w = a.w; Q = w^2*eye(2); 

v = a.v; R = v^2*eye(2); 

f = @(x) [x(2); x(2)*(1 - x(1)^2) - z(1)];
h = @(x) x;

% Jacobians
Jfx = @(x) [0 1; (2*x(1)*x(2)-1) (1 - x(1)^2)];
% Jfw = @(x)
% W = Jfw;

Jhx = @(x) [1 0; 0 1];
% Jhx = [1 0; 0 1];
% Jhv = 
% V = Jhv;


initialize(a);
order = a.order;
x_0 = a.init;
P = cov(a.x(1,:), a.x(2,:));
P_previous = P;

x_estimate = x_0;
X = []; Ptr1 = []; Ptr2 = []; E = [];
a.counter = 0;

% run(a)
%% EKF Simulation 
% x = step(a);
for i = 1:length(a.x)
    %------------------------------- Physics Update --------------------------------
    % x_focast = f(x_estimate);
    [state, measurement] = step(a);
    x_focast = state;
    % Qx = W*Q*transpose(W);
    P_focast = Jfx(x_estimate) * P_previous * transpose(Jfx(x_estimate)) + Q;
    Ptr2 = [Ptr2 trace(P_focast)];
    %------------------------------- Data Update --------------------------------
    % Rm = V*R*transpose(V);
    % K = P_focast*transpose(Jhx(x_focast)) * inv(Jhx(x_focast)*P_focast*transpose(x_focast)+R);
    K = P_focast * transpose(Jhx(x_focast)) * ...
        inv(Jhx(x_focast)*P_focast*transpose(Jhx(x_focast))+R);
    x_estimate = x_focast + K * (measurement - h(x_focast));
    E = [E (x_focast - x_estimate)];
    X = [X x_estimate];
    P_previous = (eye(order) - K*Jhx(x_estimate))*P_focast;
    Ptr1 = [Ptr1 trace(P_previous)];
end

%% Error Accumulation
% TR = Ptr2; ETR = E;
TR = TR + Ptr2;
ETR = ETR + E;

%% Plots
figure(1) 
plot(a.t, a.xn(1,:),a.t, a.xn(2,:));
hold on
plot(a.t, X(1,:),a.t, X(2,:), 'LineWidth', 3);
plot(a.t, a.zn(1,:),a.t, a.zn(2,:));
plot(a.t, a.x(1,:),a.t, a.x(2,:));
hold off
grid on
legend('State 1 + Noise','State 2 + Noise', ...
    'Estimation 1','Estimation 2', ...
    'Measurement (State 1)','Measurement (State 2)', ...
    'Correct State 1', 'Correct State 2')


figure(2) 
plot(Ptr1)
hold on
plot(Ptr2)
plot(TR./200)
plot(ETR(1,:)./200)
plot(ETR(2,:)./200)
legend('P(previous) Trace','P(focast) Trace','Avg P(focast) Trace', 'Error-1', 'Error-2')
hold off

figure(3) 
plot(X(1,:), X(2,:), 'LineWidth', 3)
hold on
plot(a.x(1,:), a.x(2,:), 'LineWidth', 3)
plot(a.zn(1,:), a.zn(2,:))
plot(a.xn(1,:), a.xn(2,:))
hold off
grid on
legend('Estimation', ...
    'Correct States', ...
    'Measurements + Noise', ...
    'States + Noise')




    

