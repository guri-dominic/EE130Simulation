classdef agent < handle
    % particle moving using a Non-linear System (ode)
    % agent(odefun), agent(odefun, initial_conditions), agent(odefun, initial_conditions, time)
    %   - process noise         w ~ N(0, W)
    %   - measurement noise     v ~ N(0, V)
    %   - time scale            t
    %   - initial conditions    init
    properties (GetAccess='private', SetAccess='private')
       var_private = 0; 
    end
    
    properties (Constant)
        var_constant = 0; 
    end
    
    properties
        % System Definition (ODE, time, initial conditions)
        ode = @(t,y) [y(2); 1*(1-y(1)^2)*y(2)-1*y(1)];
        t = (0:0.01:1);     % time (default)
        init = [1.0 2.0];
        order;
        % ODE45 Parameters
        odeopt = odeset ('RelTol', 0.00001, 'AbsTol', ...
            0.00001,'InitialStep',0.5,'MaxStep',0.5);
        % Noise Parameters
        w = 0.2;
        v = 0.15;
        % States & Measurements
        x;          % states (clean)
        xn;         % states (with noise - w)
        z;          % measurements
        zn;         % measurements (with noise - v)
        % simulation step
        counter = 1;
    end
    
    methods
        % Constructor
        function obj = agent(varargin)
            if (nargin > 0)
                % use default values
                if (nargin == 1)
                    % ode variable
                    assert(isa(varargin{1},'function_handle'));
                    obj.ode = varargin{1};
                    obj.order = length(obj.init);
                elseif (nargin == 2)
                    % ode variable
                    assert(isa(varargin{1},'function_handle'));
                    obj.ode = varargin{1};
                    % check initial conditions [change order]
                    init_conditions = varargin{2};
                    assert(isvector(init_conditions));
                    obj.init = init_conditions;
                    obj.order = length(obj.init);
                elseif (nargin == 3)
                    % ode variable
                    assert(isa(varargin{1},'function_handle'));
                    obj.ode = varargin{1};
                    % check initial conditions [change order]
                    init_conditions = varargin{2};
                    assert(isvector(init_conditions));
                    obj.init = init_conditions;
                    obj.order = length(init_conditions);
                    % time variable
                    tm = varargin{3};
                    if((length(tm) > 2) && (all(diff(tm)) >= 0))
                        obj.t = tm;
                        disp('Vector')
                    elseif((length(tm) == 2) && (tm(1) < tm(2)))
                        obj.t = linspace(tm(1),tm(2), 100);
                        disp('Start End')
                    else
                        error('Time variable incorrectly set for agent')
                    end
                end
            end
        end
        
%         function obj = agent(odefun, init_conditions)
%             assert(isa(odefun,'function_handle'));
%             assert(isvector(init_conditions));
%             obj.order = length(init_conditions);
%             obj.ode = odefun;
%         end
        
        function set.w(obj, w)
            assert(w > 0.0 && isnumeric(w));
            obj.w = w;
        end
        
        
%         function order = get.order(obj)
%             obj.order = length(obj.init);
%             order = length(obj.init);
%         end
        
        function set.v(obj, v)
            assert(v > 0.0 && isnumeric(v));
            obj.v = v;
        end
        
        function [ms] = step(obj)
            obj.counter = obj.counter + 1;
            % process with noise
            p_noise = obj.w .* randn(obj.order, 1);
            
            obj.xn(:,obj.counter) = obj.x(:,obj.counter) + p_noise;
            ms = obj.xn(:,obj.counter);
            % tm = obj.t(obj.counter);
        end
        
        function run(obj)
            obj.reset;
            for i=1:length(diff(obj.t))
                obj.step
            end
        end
        
        function reset(obj)
            obj.xn = zeros(size(obj.x));
            obj.z = zeros(size(obj.x));
            obj.zn = zeros(size(obj.x));
            obj.counter = 1;
        end
        
        function [time,x] = initialize(obj)
            obj.order = length(obj.init);
            % Noiseless Evaluations
            % Process
            [time,x] = ode45(obj.ode, obj.t, obj.init, obj.odeopt);
            obj.t = transpose(time);
            obj.x = transpose(x);
            % Measurement
        end
        
        function plotall(obj)
            % obj.initialize;
            % size(obj.x)
            % size(obj.t)
            figure
            plot(obj.t, obj.x(1,:),obj.t, obj.x(2,:));
            hold on
            plot(obj.t, obj.xn(1,:),obj.t, obj.xn(2,:));
            hold off
            figure
            plot(obj.x(1,:), obj.x(2,:))
            hold on
            plot(obj.xn(1,:), obj.xn(2,:))
            hold off
        end
    end
end

