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
        
        pos_record;
        pos_dot_record;
        theta_record;
        omega_record;
        path;

        % [System Input]
        input;


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
            %obj.input = [0.735; 0.735; 0.735; 0.735]; %<=======euilibrium
            %obj.input = [0; 0; 0; 0]; %<=======free fall
            obj.input = [0.98; 0.49; 0.98; 0.49]; %<=======float&rotate

            %Record
            obj.pos_record = obj.pos;
            obj.pos_dot_record = obj.pos_dot;
            obj.theta_record = obj.theta;
            obj.omega_record = obj.omega;
            obj.path = obj.pos;
        end

        % Main Function
        function update(obj,t, dt)
            t
            obj.dt = dt;
            obj.dt_dyn = dt;
            
            input_gamma = obj.input;
            
            for i = 1:10 % Dynamics simulate 10 times faster than input updates
                %Record
                obj.path = [obj.path obj.pos];
                obj.pos_record = [obj.pos_record obj.pos];
                obj.pos_dot_record = [obj.pos_dot_record obj.pos_dot];
                obj.theta_record = [obj.theta_record obj.theta];
                obj.omega_record = [obj.omega_record obj.omega];
    
                a = acceleration(obj, input_gamma);
                obj.omegadot = angular_acceleration(obj, input_gamma);
                obj.omega = obj.omega + obj.dt_dyn * obj.omegadot;
                
                obj.theta_dot = omega2thetadot(obj);
                obj.theta = obj.theta + obj.dt_dyn * obj.theta_dot;  
                obj.rot = [obj.theta(3); obj.theta(2);obj.theta(1)]; %system output (orientation)
                obj.pos_dot = obj.pos_dot + obj.dt_dyn * a;
                obj.pos = obj.pos + obj.dt_dyn * obj.pos_dot; %system output (position)
                %obj.omega = thetadot2omega(obj) 
                
                
            end
        end
        
        
        %----------------Quadcopter Physics----------------------
    
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

        function R = rotation(obj)

           phi = obj.theta(1);
           th = obj.theta(2);
           psi = obj.theta(3);

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
        
        function a = acceleration(obj, input_gamma)
            gravity = [0; 0; -obj.g];
            R = rotation(obj);
            T = R * thrust(obj, input_gamma);
            Fd = -obj.kd * obj.pos_dot;
            a = gravity + 1 / obj.m * T + Fd/obj.m;
        end

        function omegadot = angular_acceleration(obj, input_gamma)
            tau = torques(obj,input_gamma);
            omegadot = inv(obj.I) * (tau - cross(obj.omega, obj.I * obj.omega)); 
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
        function thetadot = omega2thetadot (obj)

            phi = obj.theta(1);
            th = obj.theta(2);
            psi = obj.theta(3);

            matrix = [[1, 0, -sin(th)];
                      [0, cos(phi), cos(th)*sin(phi)];
                      [0, -sin(phi), cos(th)*cos(phi)];];
            thetadot = inv(matrix) * obj.omega;
        end
        %-------------------------------------------------------
        
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