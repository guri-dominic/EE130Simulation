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
        ode = @(t,y) [y(2); 1*(1-y(1)^2)*y(2)-1*y(1)];  % Van der Pol Oscill.
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
        % simulation step
        counter = 1;
        % measurement model
        h = @(x) x;
        z;          % measurements
        zn;         % measurements (with noise - v)
    end
    
    methods
        % Constructor
        function obj = agent(varargin)
            if (nargin > 0)
                % use default values
% --------------------------- 1 Variables (ode) ----------------------------
                if (nargin == 1)
                    % ode variable
                    assert(isa(varargin{1},'function_handle'));
                    obj.ode = varargin{1};
                    obj.order = length(obj.init);
% ------------------------ 2 Variables (ode, init) -------------------------
                elseif (nargin == 2)
                    % ode variable
                    assert(isa(varargin{1},'function_handle'));
                    obj.ode = varargin{1};
                    % check initial conditions [change order]
                    init_conditions = varargin{2};
                    assert(isvector(init_conditions));
                    obj.init = init_conditions;
                    obj.order = length(obj.init);
% ---------------------- 3 Variables (ode, init, time) ----------------------
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
% ------------------- 4 Variables (ode, init, time, noise) ------------------
                elseif (nargin == 4)
                    % 1. ode variable
                    assert(isa(varargin{1},'function_handle'));
                    obj.ode = varargin{1};
                    % 2. check initial conditions [change order]
                    init_conditions = varargin{2};
                    assert(isvector(init_conditions));
                    obj.init = init_conditions;
                    obj.order = length(init_conditions);
                    % 3. time variable
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
                    % 4. noise
                    noise = varargin{4};
                    assert(isvector(noise) && (length(noise) == 2));
                    obj.w = noise(1);
                    obj.v = noise(2);
% --------- 5 Variables (ode, init, time, noise, measurement_model) ---------
                elseif (nargin == 4)
                    % 1. ode variable
                    assert(isa(varargin{1},'function_handle'));
                    obj.ode = varargin{1};
                    % 2. check initial conditions [change order]
                    init_conditions = varargin{2};
                    assert(isvector(init_conditions));
                    obj.init = init_conditions;
                    obj.order = length(init_conditions);
                    % 3. time variable
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
                    % 4. noise
                    noise = varargin{4};
                    assert(isvector(noise) && (length(noise) == 2));
                    obj.w = noise(1);
                    obj.v = noise(2);
                    % 5. measurement model
                    meas_model = varargin{5};
                    assert(isa(meas_model,'function_handle'));
                    obj.h = meas_model;
                end
            end
        end
        
        function set.w(obj, w)
            assert(w > 0.0 && isnumeric(w));
            obj.w = w;
        end
        
        function set.v(obj, v)
            assert(v > 0.0 && isnumeric(v));
            obj.v = v;
        end
        
        function [state, meas] = step(obj)
            obj.counter = obj.counter + 1;
            index = obj.counter;
            % process with noise
            p_noise = obj.w .* randn(obj.order, 1);
            
            obj.xn(:, index) = obj.x(:, index) + p_noise;
            state = obj.xn(:, index);
            % tm = obj.t(index);
            meas = obj.measure(index);
        end
% ------------ Measurement Model Function ---------------
        function meas = measure(obj, index)
            obj.z(:, index) = obj.h(obj.x(:, index));
            obj.zn(:, index) = obj.z(:, index) + obj.v .* randn(obj.order, 1);
            meas = obj.zn(:, index);
            % size(obj.z)
        end
% ------------ Take All Step()  ---------------
        function run(obj)
            obj.reset;
            for i=1:length(diff(obj.t))
                obj.step;
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
            plot(obj.t, obj.zn(1,:),obj.t, obj.zn(2,:));
            hold off
            legend('States','States + Noise(w)','Measurement + Noise(v)')
            
            figure
            plot(obj.x(1,:), obj.x(2,:))
            hold on
            plot(obj.zn(1,:), obj.zn(2,:))
            plot(obj.xn(1,:), obj.xn(2,:))
            legend('Position','Position + Noise(w)','Measurement + Noise(v)')
            hold off
        end
    end
end

