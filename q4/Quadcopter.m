classdef Quadcopter < handle
    
    % Define robot fixed parameters
    properties (Access=private, Constant)
        
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];

        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];

        % Parameters from Question 1
        m = 0.3;
        I = [[1 0 0];[0 1 0];[0 0 0.4]];
        g = 9.8;
        kd = 0.2;
        k = 1;
        L = 0.25;
        b = 0.2;

        Q = diag([1;1;1;1;1;1;1;1;1;1;1;1]); %12*1
        R = diag([1;1;1;1]);


    end
    
    % Define robot variable parameters (incl. state, output, input, etc)
    properties %(Access=private)   
        % plotting
        ax (1,1) matlab.graphics.axis.Axes;
        % Robot parameters needed for plotting
        pos (3,1) double; % 3D position: x-y-z 
        rot (3,1) double; % 3D orientation: yaw-pitch-roll 
        pos_dot (3,1) double;
        rot_dot (3,1) double;
        omega (3,1) double;
        omegadot (3,1) double;
        theta (3,1) double; % 3D orientation: roll-pitch-yaw
        theta_dot (3,1) double;
        dt;
        dt_dyn;

        % [System Input]
        input;

        % [state space matrices]
        A; %state matrix
        B; %input-to-state matirx
        C; %state-to-output matrix
        D; %feedthrough matrix

        %define state
        state;
        state_dot;

        %define equilbrium state & input
        state_equil;
        input_equil;

        % Q3
        ref; % quardcopter check points
        marker; % quardcopter check point marks
        eigenvalues;
        K;
        time;
        time_record;
        reachable;
        path; % matrix storing path
        circle;
        circle_i;
        landing_speed; % matrix storing landing speed 
        input_record; % matrix storing 4 inputs over time
        
        %Q4 noise
        mean_value; 
        std_deviation; 
        noise_amplitude;

    end    

    methods
        % Class constructor

        function obj = Quadcopter(ax)

            %initialising variables (starting state)
            obj.pos = [0; 0; 0]; % 3D position: x-y-z  
            obj.rot = [0; 0; 0]; % 3D orientation: yaw-pitch-roll
            obj.theta = [0; 0; 0]; % 3D orientation: roll-pitch-yaw
            obj.theta_dot = [0; 0; 0];
            obj.omega = [0; 0; 0];
            obj.omegadot = [0; 0; 0];
            obj.ax = ax;

            % Setting Input (starting input)
            obj.input = [0; 0; 0; 0];

            %initialise state
            %obj.state = [0;0;0; %pos
                         %0;0;0; %pos_dot
                         %0;0;0; %theta
                         %0;0;0]; %omega
    
            obj.state_dot = [0;0;0;
                             0;0;0;
                             0;0;0;
                             0;0;0];
        
            %equilibrium state
            obj.state_equil = [0;0;5;
                               0;0;0;
                               0;0;0;
                               0;0;0];
    
            obj.input_equil = [0.735; 0.735; 0.735; 0.735]; % Quadcopter Floating

            obj.state = [obj.pos; obj.pos_dot; obj.theta; obj.omega]; %initialise state

            linearisation(obj); % update A and B
            obj.K = lqr(obj.A, obj.B, obj.Q, obj.R);
            disp("obj.K=")
            disp(obj.K)

            %obj.reachable = checkReachability(obj.A, obj.B);
            %disp(obj.reachable)
            
            % Initialise variables for Q3
             % Checking points (each column is a point)
            obj.ref = [0,   0,   0,   0,   0,   0,  2.5,  2.5;
                       0, 2.5,   0,-2.5,   0, 2.5,  2.5,  2.5;
                       5,   5, 7.5,   5, 2.5,   5,  2.5,    0];

            obj.ref = [obj.ref; zeros(9,8)]; % padding to make the matrix ref 12*8 (only pos matter)

            obj.marker = [0; 0; 0; 0; 0; 0; 0; 0]; % each time 

            obj.eigenvalues = linspace(0.85,0.95,12);

            obj.time = 0;
            obj.time_record = 0;

            obj.path = obj.pos;

            obj.circle = createCircle(obj);

            obj.circle_i = 1;

            obj.landing_speed = obj.pos_dot;

            obj.input_record = obj.input;

            %Q4 Noise
            obj.mean_value = 0; 
            obj.std_deviation = 0.5; 
            obj.noise_amplitude = 0.01;
    

        end    

        %===========================Main Function==========================
        function update(obj,t,dt)

            obj.time = t;
            obj.dt = dt;
            obj.dt_dyn = 2 * dt;% dynamics simulation step size
            tolerance = 0.05;
            obj.state(1:3) = obj.pos;
            obj.state(4:6) = obj.pos_dot;
            obj.state(7:9) = obj.theta;
            obj.state(10:12) = obj.omega;

            obj.path = [obj.path obj.pos];
            obj.input_record = [obj.input_record obj.input];
            %disp(obj.path)

            
            if obj.marker(1) == 0
                ref_indexed = obj.ref(:,1); % Set target as Point 1
                disp("flying to Point 1")
                fsf_controller(obj, ref_indexed); % Adjust input according to current position
                moving(obj);
                if sqrt(sum((ref_indexed(1:3)-obj.pos).^2)) < tolerance
                    obj.marker(1) = 1;
                    disp("Point 1 approached")
                    obj.time_record = obj.time;
                end

            elseif obj.marker(2) == 0 && obj.marker(1) == 1 %Not yet approached Point 2
                if obj.time < obj.time_record+5
                    ref_indexed = obj.ref(:,1);
                    disp("staying at Point 1 for 5 seconds")
                    fsf_controller(obj, ref_indexed); % Stay in Point 1 for 5 seconds
                    moving(obj);

                elseif obj.time >= obj.time_record+5 % After 5 seconds
                    ref_indexed = obj.ref(:,2); % Set target as Point 2
                    disp("flying to Point 2")
                    fsf_controller(obj, ref_indexed);
                    moving(obj);
                    if sqrt(sum((ref_indexed(1:3)-obj.pos).^2)) < tolerance
                        obj.marker(2) = 1;
                        disp("Point 2 approached")
                        obj.time_record = obj.time;
                    end
                end

            elseif obj.marker(3)+obj.marker(4)+obj.marker(5)+obj.marker(6) < 4 && obj.marker(2) == 1 %Not yet starting circle
                if obj.time < obj.time_record + 0
                    ref_indexed = obj.ref(:,2);
                    %disp("staying at point 2 for stability")
                    fsf_controller(obj, ref_indexed); % Stay in Point 2 for 0 seconds for stability
                    moving(obj);
                elseif obj.time >= obj.time_record + 0 % After 0 seconds
                    %Start flying in circle
                    disp("Flying in circle while passing Point 3, 4, 5, 6")
                    if obj.circle_i <= size(obj.circle, 2) 
                        ref_indexed = obj.circle(:,obj.circle_i);
                        fsf_controller(obj, ref_indexed);
                        moving(obj);
                        if sqrt(sum((ref_indexed(1:3)-obj.pos).^2)) < tolerance
                            obj.circle_i = obj.circle_i + 1;
                            %during flying in circle, reach point 3, 4, 5
                            if sqrt(sum((obj.ref(1:3,3)-obj.pos).^2)) < tolerance
                                obj.marker(3) = 1;
                                disp("Point 3 approached")
                                
                            end
                            if sqrt(sum((obj.ref(1:3,4)-obj.pos).^2)) < tolerance
                                obj.marker(4) = 1;
                                disp("Point 4 approached")
                            end
                            if sqrt(sum((obj.ref(1:3,5)-obj.pos).^2)) < tolerance
                                obj.marker(5) = 1;
                                disp("Point 5 approached")
                            end
                        end
                    elseif sqrt(sum((obj.ref(1:3,6)-obj.pos).^2)) < tolerance
                        obj.marker(6) = 1;
                        disp("Point 6 approached")
                    end
                end
            

            elseif obj.marker(7) == 0 && obj.marker(6) == 1
                ref_indexed = obj.ref(:,7); % Set target as Point 7
                disp("flying to Point 7")
                fsf_controller(obj, ref_indexed);
                moving(obj);
                if sqrt(sum((ref_indexed(1:3)-obj.pos).^2)) < tolerance
                    obj.marker(7) = 1;
                    disp("Point 7 approached")
                end

            elseif obj.marker(8) == 0 && obj.marker(7) == 1
                ref_indexed = obj.ref(:,8); % Set target as Point 8
                ref_indexed(4) = 0; % Set the x-direction speed to 0m/s
                ref_indexed(5) = 0; % Set the y-direction speed to 0m/s
                ref_indexed(6) = -0.04; %Set the z-direction speed to 0.1m/s
                obj.landing_speed = [obj.landing_speed obj.pos_dot]; % record landing speed
                disp("Landing at Point 8 at 0.1m/s")
                fsf_controller(obj, ref_indexed);
                moving(obj);
                if sqrt(sum((ref_indexed(1:3)-obj.pos).^2)) < tolerance
                    obj.marker(8) = 1; 
                    disp("Landed at Point 8 successfully!")
                    obj.time_record = obj.time;
                    disp("Totoal time =")
                    disp(obj.time_record)
                end

            end

        end
        %==================================================================
        function noise = wind_noise(obj)
            %mean_value = 0;         
            %std_deviation = 1;     
            %noise_amplitude = 0.1; 
            
            % Generate Gaussian noise with the specified mean and standard deviation
            noise = obj.mean_value + obj.std_deviation * obj.noise_amplitude * randn(1, 3);
            noise = noise.'; %noise is 3*1 matrix
        end

        function circle = createCircle(obj)
            radius = 2.5;
            height = 5;
            num_points = 101; % num_point should be 4n+1
            
            % Initialize the matrix to store the points
            circle = zeros(3, num_points);
            
            % Calculate the angles for the points on the circle
            angles = linspace(0.5*pi, 2.5*pi, num_points);
            
            % Calculate the x, y, and z coordinates for each point
            circle(1, :) = zeros(1, num_points); % x-coordinate is always 0
            circle(2, :) = radius * sin(angles);
            circle(3, :) = height - radius * cos(angles);

            circle = [circle; zeros(9,101)];
            % Display the resulting matrix
            disp(circle);
        end

        function Reachable = checkReachability(A, B)
        
            % Compute the reachability matrix
            Wr = reachabilityMatrix(A, B);
        
            % Check if the matrix is full rank
            if rank(Wr) == size(A, 1)
                Reachable = true;
            else
                Reachable = false;
            end
        end

        function Wr = reachabilityMatrix(A, B)

            % Number of states (number of rows of A)
            Wr = size(A, 1);
            % Initialize reachability matrix
            Wr = B;
            % Compute reachability matrix
            for i = 2:n
                Wr = [Wr, A^(i-1) * B];
            end
        end

        function moving(obj)
            disp("obj.input=")
            disp(obj.input)
            disp("obj.pos=")
            disp(obj.pos)
            obj.omega = obj.omega + obj.dt_dyn * angular_acceleration(obj, obj.omega, obj.input);
            %disp(obj.omega)
            obj.theta_dot = omega2thetadot(obj, obj.theta, obj.omega);
            obj.theta = obj.theta + obj.dt_dyn * obj.theta_dot;
            obj.pos_dot = obj.pos_dot + obj.dt_dyn * acceleration(obj, obj.theta, obj.pos_dot, obj.input);
            obj.pos = obj.pos + obj.dt_dyn * obj.pos_dot;
            obj.omega = thetadot2omega(obj);

            %Noise
            obj.pos = obj.pos + wind_noise(obj);
            obj.pos_dot = obj.pos_dot + wind_noise(obj);
            obj.theta = obj.theta + wind_noise(obj);
            obj.omega = obj.omega + wind_noise(obj);
        end

        % Full State Feedback Controller
        function fsf_controller(obj, ref_indexed)

            %obj.K = place(obj.A, obj.B, obj.eigenvalues);
  
            % e: 12*1 Matrix, the first 3 elements are position errors and the second 3 elements are velocity errors.
            e = obj.state - ref_indexed; % error = current postion - reference position
            disp("error=")
            disp(e)
            raw_input = obj.input_equil - obj.K * e;
            % Limit the control input to the range [-1.5, +1.5]
            obj.input = max(min(raw_input, 1.5), -1.5);
        end

        % Linearisation
        function linearisation(obj)
            % [System Input]
            syms gamma1 gamma2 gamma3 gamma4
            syms pos_x pos_y pos_z pos_dot_x pos_dot_y pos_dot_z theta_x theta_y theta_z omega_x omega_y omega_z
            syms x1dot_x x1dot_y x1dot_z x2dot_x x2dot_y x2dot_z x3dot_x x3dot_y x3dot_z x4dot_x x4dot_y x4dot_z

            input_gamma = [gamma1; gamma2; gamma3; gamma4]; 

            x1 = [pos_x; pos_y; pos_z]; %pos
            x2 = [pos_dot_x; pos_dot_y; pos_dot_z]; %pos_dot
            x3 = [theta_x; theta_y; theta_z]; %theta(roll; pitch; yaw)
            x4 = [omega_x ;omega_y ;omega_z]; %omega

            x1dot = [x1dot_x; x1dot_y; x1dot_z]; %pos_dot
            x2dot = [x2dot_x; x2dot_y; x2dot_z]; %acceleration
            x3dot = [x3dot_x; x3dot_y; x3dot_z]; %theta_dot
            x4dot = [x4dot_x; x4dot_y; x4dot_z]; %omega_dot

            % State
            x = [x1; x2; x3; x4]; 

            % Dynamics
            x1dot = x2;
            x2dot = acceleration(obj, x3 , x2, input_gamma);
            x3dot = omega2thetadot(obj, x3, x4);
            x4dot = angular_acceleration(obj, x4, input_gamma);

            x_dot = [x1dot; x2dot; x3dot; x4dot];
            
            % Calculating Jacobian
            Aj = jacobian(x_dot,x);
            Bj = jacobian(x_dot,input_gamma);

            % Replacing Equilibrium Point

            equilibrium_x = [0;0;5; % position(can be any value)
                            0;0;0; % linear velocity
                            0;0;0; % angle
                            0;0;0]; % angular velocity

            equilibrium_u = [0.735; 0.735; 0.735; 0.735]; %floating

            A_state = subs(Aj, x, equilibrium_x);
            A_state = double(subs(A_state, input_gamma, equilibrium_u));
            B_state = subs(Bj,x, equilibrium_x);
            B_state = double(subs(B_state, input_gamma, equilibrium_u));

            % The output equation is already linear
            C_state = eye(size(Aj));
            D_state = zeros(size(Bj));

            sys = ss(A_state, B_state, C_state, D_state);
            %sys_discrete = c2d(sys, obj.dt, 'zoh');

            %update state matrices
            obj.A = sys.A;
            obj.B = sys.B;
            obj.C = sys.C;
            obj.D = sys.D;
            disp(obj.A);
            disp(obj.B);
        end
        

        %----------------------Quadcopter Physics--------------------------
        function T = thrust(obj, input_gamma)
            T = [0; 0; obj.k * sum(input_gamma)];
        end

        function tau = torques(obj,input_gamma)
            tau = [
                obj.L * obj.k * (input_gamma(1) - input_gamma(3))
                obj.L * obj.k * (input_gamma(2) - input_gamma(4))
                obj.b * (input_gamma(1) - input_gamma(2) + input_gamma(3) - input_gamma(4))
                ];
        end

        function R = rotation(obj, angle)

           phi = angle(1);
           th = angle(2);
           psi = angle(3);

           Rx = [[1, 0, 0];
                 [0, cos(phi), -sin(phi)];
                 [0, sin(phi), cos(phi)]];
           Ry = [[cos(th), 0, sin(th)];
                 [0, 1, 0];
                 [-sin(th), 0, cos(th)]];
           Rz = [[cos(psi), -sin(psi), 0];
                 [sin(psi), cos(psi), 0];
                 [0, 0, 1]];
           R = Rz*Ry*Rx;
        end
        
        function a = acceleration(obj, angle, pos_dot, input_gamma)
            gravity = [0; 0; -obj.g];
            R = rotation(obj, angle);
            T = R * thrust(obj, input_gamma);
            Fd = -obj.kd * pos_dot;
            a = gravity + 1 / obj.m * T + Fd/obj.m;
        end

        function omegadot = angular_acceleration(obj, omega, input_gamma)
            tau = torques(obj,input_gamma);
            omegadot = inv(obj.I) * (tau - cross(omega, obj.I * omega)); 
        end

        function omega = thetadot2omega (obj)

            phi = obj.theta(1);
            th = obj.theta(2);
            psi = obj.theta(3);

            matrix = [[1, 0, -sin(th)];
                      [0, cos(phi), cos(th)*sin(phi)];
                      [0, -sin(phi), cos(th)*cos(phi)];];
            omega = matrix * obj.rot_dot;
        end
        function thetadot = omega2thetadot (obj, angle, omega)

            phi = angle(1);
            th = angle(2);
            psi = angle(3);

            matrix = [[1, 0, -sin(th)];
                      [0, cos(phi), cos(th)*sin(phi)];
                      [0, -sin(phi), cos(th)*cos(phi)];];
            thetadot = inv(matrix) * omega;
        end

        %------------------------------------------------------------------
        
        function plot(obj,t)
            %create middle sphere
            [X Y Z] = sphere(8);
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.ax,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            rot_mat           = eul2rotm(obj.rot.');
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = rot_mat*rotorPosBody;
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.ax,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end

            obj.ax.Title.String = ['Time = ', num2str(obj.time, '%3f'), 's'];
            plot3(obj.ax, obj.path(1,:), obj.path(2,:), obj.path(3,:)); 
        end
    end
end