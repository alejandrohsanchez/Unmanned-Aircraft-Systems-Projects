clc
clear;
close all;

%initialize everything
t = 0; % This is the initial time (0 seconds)
time_end = 200; % This is the final time (200 seconds)
dt = 0.01; % Time step

i_max = time_end/dt; % This is the last index

%Drone structure: p_north, p_east, altitude, altitude_d, airspeed, course, roll, roll_d
drone.s = [0,0,0,0,2,0,0,0];
drone.data.x = zeros(8,i_max);

%u = height, AirSpeed, RollAngle
drone.u.Height = 10; % This is the drone altitude
drone.u.AirSpeed = 5; % This is the drone airspeed
drone.u.RollAngle = 0; % This is the drone roll angle
drone.u.Xi_c = 0; % This is the course heading

drone.data.u = zeros(4,i_max);

%What do these do?
% These variables will help calculate the drone flight dynamics, which
% include discrete time approximations that tell us information about where
% the drone is heading. These variables will be entered in equations to
% calculate pn dot, pe dot, Xi double dot, Va dot, and a few more. These
% dot variables are necessary to measure the discrete time approximations.
drone.param.bX = 0.09;
drone.param.br = 0.6;
drone.param.bdr = 1.1;
drone.param.bdH = 2;
drone.param.bH = .6;
drone.param.bVa = .5;
drone.param.max_bank = deg2rad(45);

Flight_Plan.State = 0;

sim = true;

i = 1;
while sim == true
    
    
    %Move aircraft information from the x state vector to more readable
    %variables.  Label each variable.
    pn = drone.s(1); % Drone position north
    pe = drone.s(2); % Drone position east
    h = drone.s(3);  % Drone height (down)
    h_d = drone.s(4); % Drone altitude derivative
    Va = drone.s(5); % Drone airspeed (vertical velocity)
    Xi = drone.s(6); % Drone course heading (initial)
    roll = drone.s(7); % Drone roll
    roll_d = drone.s(8); % Drone roll time derivative
    
    %Default command values
    drone.u.Height = 10; 
    drone.u.AirSpeed = 5;
    drone.u.RollAngle = 0; 
    
    %% State Machine
    %At each state, calculate the state output and check to see if we
    %trigger a change to another state
    
    % This state machine contains if conditionals that check the location
    % of the UAV. If the UAV has certain height, airspeed, and state
    % values, the state machine will trigger the UAV to move to the next
    % stage. This is important because we want to tell our UAV where to go.
    if (Flight_Plan.State == 0)
        %Output
        drone.u.Height = 18;  
        drone.u.AirSpeed = 5;
        drone.u.Xi_c = 0;
        
        % CHECK THIS --------------------------------------
        
        %The drone should be going at 5 (airspeed) up to 18
        %(altitude) and heading north (north,east,south,west)
        
        %Trigger
        if (h>17) %If height is greater than 17 
            Flight_Plan.State = 1;
        end
    elseif (Flight_Plan.State == 1)
        %Output
        drone.u.Height = 23;
        drone.u.AirSpeed = 7;
        drone.u.Xi_c = deg2rad(30);
        
        % CHECK THIS --------------------------------------
        
        %The drone should be going at 7 (airspeed) up to 23
        %(altitude) and northeast 30 degrees (north,east,south,west)
        
        %Trigger
        if (pe>80) %If position east is greater than 80
            Flight_Plan.State = 2;
        end
    elseif (Flight_Plan.State ==2)
        %Output
        drone.u.Height = 30; 
        drone.u.AirSpeed = 7;
        drone.u.Xi_c = deg2rad(-30);
        
        %The drone should be going at 7 (airspeed) up to 30
        %(altitude) and northwest 30 degrees(north,east,south,west)
        
        %Trigger
        if (pe<-80) %If position east is less than -80
            Flight_Plan.State = 3;
        end
    elseif (Flight_Plan.State == 3)
        %Output
        drone.u.Height = 8; 
        drone.u.AirSpeed = 4;
        drone.u.Xi_c = deg2rad(-30);
        
        %The drone should be going at 4 (airspeed) up to 8
        %(altitude) and northwest 30 degrees (north,east,south,west)
        
        %Trigger
        if (h<9) %If height is less than 9 
            Flight_Plan.State = 4;
        end
    elseif (Flight_Plan.State == 4)
        %Output
        prev_speed = Va;
        prev_alt = h;
        new_speed = prev_speed - 5*dt;
        if (new_speed < 0)
            new_speed = 0;
        end
        new_alt = h-60*dt;
        if (new_alt < 0)
            new_alt = 0;
        end
        
        %Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = new_speed;
        drone.u.Xi_c = deg2rad(0);
        
        %The drone should be going at 3.9813 (airspeed) to 8.3953
        %(altitude) and north (north,east,south,west)
        
        %Trigger
        if (h<1) %If height is less than 1 
            Flight_Plan.State = 5;
        end
    elseif (Flight_Plan.State == 5)
        %Output
        drone.u.Height = 0; 
        drone.u.AirSpeed = 0;
        drone.u.Xi_c = deg2rad(0);
        
        %The drone should be going at 0 (airspeed) up to 0
        %(altitude) and north (north,east,south,west)
        
        %Trigger
        %Last State, no transition
    end
    
    %% Calculate drone dynamics
        
    % Implement P control for bank angle
    drone.u.RollAngle = calc_RollAngle(drone);
    
    %update the aircraft
    pnd = Va*cos(Xi); % This is the UAV's velocity heading north
    ped = Va*sin(Xi); % This is the UAV's velocity heading east
    
    % h_dd is h double dot, which represents the acceleration of altitude
    % (height) - rate of change in velocity moving up and down
    h_dd = drone.param.bdH*(-h_d)+drone.param.bH*(drone.u.Height-h);
    
    % Va_d is the time derivative (similar to h_dd) that measures the
    % change in airspeed.
    Va_d = drone.param.bVa*(drone.u.AirSpeed-Va);
    
    if (Va > 2)
        Xi_d = 9.81/Va*tan(roll);
    else
        Xi_d = 0;
    end
    
    % roll_dd is the rate of change of roll speed (acceleration value of
    % roll)
    roll_dd = drone.param.bdr*(-roll_d)+drone.param.br*(drone.u.RollAngle-roll);
    
    % The structs below refer to the drone structure:
    drone.s(1) = pn+pnd*dt; % position north update
    drone.s(2) = pe+ped*dt; % position east update
    drone.s(3) = h+h_d*dt; % altitude update
    drone.s(4) = h_d+h_dd*dt; % altitude time derivative update
    drone.s(5) = Va+Va_d*dt; % airspeed update
    Xi = Xi+Xi_d*dt;
    if (Xi > 2*pi)
        Xi = Xi-2*pi;
    elseif (Xi < -2*pi)
        Xi = Xi+2*pi;
    end
    drone.s(6) = Xi; % course heading update
    drone.s(7) = roll+roll_d*dt; % roll update
    drone.s(8) = roll_d + roll_dd*dt; % roll time derivative update
    
    % Updating all the drone data values to match the drone structure
    % (variables aforementioned)
    drone.data.x(:,i) = drone.s;
    
    % Updating all the u values to match the state machine's results from
    % earlier in the code.
    drone.data.u(:,i) = [drone.u.Height,drone.u.AirSpeed,drone.u.RollAngle,drone.u.Xi_c];

    t = t+dt;
    i = i+1;
    
    if (i>=i_max)
        sim = false;
    end
end


t = 0:dt:time_end;
i = i-1;


%% plot everything
figure;
plot(drone.data.x(2,1:i),drone.data.x(1,1:i));
axis equal;
title('Drone Flight Path')
legend('UAV path history')
subtitle('This plot depicts the drone flight history during its flight')

figure;
plot(drone.data.x(3,1:i));
hold on;
plot(drone.data.u(1,1:i),'r');
title('Drone Waypoint Following for Altitude')
legend('UAV actual altitude', 'Waypoint Instruction')
subtitle('This plot depicts the drone altitude change history during its flight')

figure;
plot(drone.data.x(7,1:i)); hold on;
plot(drone.data.u(3,1:i),'r');
title('Drone Waypoint Following for Changes in Roll')
legend('UAV actual roll', 'Waypoint Instruction')
subtitle('This plot depicts the drone roll change history during its flight.')

figure;
plot(drone.data.x(6,1:i)); hold on;
plot(drone.data.u(4,1:i),'r');
title('Drone Waypoint Following for Changes in Course Heading')
legend('UAV actual course heading', 'Waypoint Instruction')
subtitle('This plot depicts the drone course heading change history during its flight')

%Implement P controller for calculating a roll angle from a given heading
function RollAngle = calc_RollAngle(drone)
    Xi_c = drone.u.Xi_c;

    %Why do we have this?  Is there a better function we can use?
    % The reason we would like this is to ensure that we are giving values
    % that are greater than 0 to the state machine. This will make sure
    % that our UAV goes in the correct orientation. Another way to make
    % this better is to make all the roll angles positive (or design the
    % state machine such that we only input positive angles). This
    % conditional adds 2pi to make sure the angle is properly read as a
    % positive angle in the correct direction.
    if Xi_c < 0
        Xi_c = Xi_c + 2*pi;
    end
    
    %Why do we have this condition? hint: what happens if we're heading
    %north-east and we want to turn north-west
    % This is to ensure that we are reading positive angles and to
    % correctly read the course heading angle. If we were heading
    % northeast and we wanted to turn to head northwest, we could simply
    % add pi/2 (90 degrees) to change our heading angle. This is assuming
    % that the conditionals below iterate and correct for our angle
    % reading prior to making the calculation.
    if Xi_c-drone.s(6) < -pi
        Xi_c = Xi_c + 2*pi;
    end
    if Xi_c-drone.s(6) > pi
        Xi_c = Xi_c - 2*pi;
    end
    
    RollAngle = drone.param.bX*(Xi_c-drone.s(6));

    %Why do we have this here?
    % This conditional's purpose is to check if our current roll angle is
    % larger than our maximum roll angle (max_bank) permitted. From what it
    % seems that the maximum roll angle permitted is there to keep the UAV
    % from banking too far as to not let it lose stability and accidentally
    % crash (if the state machine were to push it to this limit). If the
    % maximum roll angle were to be exceeded, this would just set the
    % command for the roll angle to be lowered to the maximum permitted
    % roll angle (as a safety).
    if RollAngle > drone.param.max_bank
        RollAngle = drone.param.max_bank;
    end
    % The same logic can be applied here. If the UAV were instructed to fly
    % and perform a bank that exceeds the lowest maximum roll angle (the
    % opposite direction), the roll angle would be reduced to the smallest
    % bank angle (negative maximum bank) to maintain stability.
    if RollAngle < -drone.param.max_bank
        RollAngle = -drone.param.max_bank;
    end
end
