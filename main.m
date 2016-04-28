% 
[t,y] = ode15s('vdp',[0 3000],[2; 0]);
plot(t,y(:,1),'o');
title('Solution of van der Pol Equation, mu = 1000');
xlabel('time t');
ylabel('solution y(:,1)');