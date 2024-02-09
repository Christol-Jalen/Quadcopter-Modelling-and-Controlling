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
        
        %Record
        pos_record;
        pos_dot_record;
        theta_record;
        omega_record;
        path;

    end    

    methods
        % Class constructor

        function obj = Quadcopter(ax)

            %initialising variables
            obj.pos = [0; 0; 5]; % 3D position: x-y-z  
            obj.rot = [0; 0; 0]; % 3D orientation: yaw-pitch-roll
            obj.theta = [0; 0; 0]; % 3D orientation: roll-pitch-yaw
            obj.theta_dot = [0; 0; 0];
            obj.ax = ax;

            % Setting Input [Comment & Uncomment the following code to simulate different scenarios]
            %obj.input = [0.735; 0.735; 0.735; 0.735]; %<=======float
            %obj.input = [0; 0; 0; 0]; %<=======free fall
            obj.input = [0.98; 0.49; 0.98; 0.49]; %<=======float&rotate

            %initialise state
            obj.state = [0;0;0; %pos
                         0;0;0; %pos_dot
                         0;0;0; %theta
                         0;0;0]; %omega
    
            obj.state_dot = [0;0;0;
                             0;0;0;
                             0;0;0;
                             0;0;0];
        
            %equilibrium state
            obj.state_equil = [0;0;0;
                               0;0;0;
                               0;0;0;
                               0;0;0];
    
            obj.input_equil = [0.735; 0.735; 0.735; 0.735]; % Quadcopter Floating

            obj.state = [obj.pos; obj.pos_dot; obj.theta; obj.omega]; %update state

            obj.path = obj.pos;

            linearisation(obj);
        end    

        % Main Function
        function update(obj,t, dt)
            t

            %Record
            obj.pos_record = [obj.pos_record obj.pos];
            obj.pos_dot_record = [obj.pos_dot_record obj.pos_dot];
            obj.theta_record = [obj.theta_record obj.theta];
            obj.omega_record = [obj.omega_record obj.omega];
            obj.path = [obj.path obj.pos];

            obj.dt = dt;
            obj.dt_dyn = dt;
   
            obj.state_dot = obj.dt_dyn*((obj.A*(obj.state - obj.state_equil) + obj.B*(obj.input - obj.input_equil))); 

            obj.state = obj.state + obj.state_dot;

            obj.pos = obj.state(1:3);
            obj.pos_dot = obj.state(4:6);
            obj.theta = obj.state(7:9);
            obj.omega = obj.state(10:12);

            obj.rot = [obj.theta(3); obj.theta(2); obj.theta(1)];

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

            equilibrium_x = [0;0;0; % position
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
            obj.A = sys.A;
            obj.B = sys.B;
            obj.C = sys.C;
            obj.D = sys.D;

            A_display = obj.A
            B_display = obj.B
            
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
        
        function plot(obj)
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
            
        end
    end
end