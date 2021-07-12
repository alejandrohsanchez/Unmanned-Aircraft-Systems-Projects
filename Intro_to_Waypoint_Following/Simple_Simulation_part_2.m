clc
clear;
close all;

%initialize everything
mySim = initialize_simulation();

drone = initialize_drone();

drone.data.x = zeros(8,mySim.idx_end);
drone.data.u = zeros(4,mySim.idx_end);

Flight_Plan = initialize_flight_plan();

mySim.sim_flag = true;

i = 1;
while mySim.sim_flag == true
    
    %Move aircraft information from the x state vector to more readable
    %variables.  Label each variable.
    pn = drone.s(1);
    pe = drone.s(2);
    h = drone.s(3);
    h_d = drone.s(4);
    Va = drone.s(5);
    Xi = drone.s(6);
    roll = drone.s(7);
    roll_d = drone.s(8);
    
    %Default command values
    drone.u.Height = 10; 
    drone.u.AirSpeed = 5;
    drone.u.RollAngle = 0; 
    
    %% State Machine
    %At each state, calculate the state output and check to see if we
    %trigger a change to another state
    if (Flight_Plan.State == 0)
        %Output
        drone.u.Height = 18;  
        drone.u.AirSpeed = 9;
        drone.u.Xi_c = 0;
        
        %The drone should be going at _____ (airspeed) up to ____
        %(altitude) and heading ______ (north,east,south,west)
        
        %Trigger
        if (h>17) %If __________ 
            Flight_Plan.State = 1;
            mySim.start_loop_time = mySim.t; %Start timer
        end
    elseif (Flight_Plan.State == 1)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 30;
        drone.u.AirSpeed = 5;
        drone.u.Xi_c = deg2rad(30);
        
        %The drone should be going at _____ (airspeed) up to ____
        %(altitude) and _____ (north,east,south,west)
        
        %Trigger
        if (mySim.loop_time > 50)
            Flight_Plan.State = 2;
            mySim.start_loop_time = mySim.t; %Restart timer
        end
        
    elseif (Flight_Plan.State ==2)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 30; 
        drone.u.AirSpeed = 5;
        drone.u.Xi_c = deg2rad(150);
        
        %The drone should be going at _____ (airspeed) up to ____
        %(altitude) and _____ (north,east,south,west)
        %Trigger
        if (mySim.loop_time > 50)
            Flight_Plan.State = 3;
            mySim.start_loop_time = mySim.t; %Restart timer
        end
        
    %State 3   
    elseif (Flight_Plan.State ==3)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 30; 
        drone.u.AirSpeed = 5;
        drone.u.Xi_c = deg2rad(300);  
        
        %Trigger
        if (mySim.loop_time > 60)
            Flight_Plan.State = 4;
            mySim.start_loop_time = mySim.t; %Restart timer
        end
    
    % State 4
    elseif (Flight_Plan.State == 4)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 20; 
        drone.u.AirSpeed = 5;
        drone.u.Xi_c = deg2rad(90);  
        
        %Trigger
        if (mySim.loop_time > 50)
            Flight_Plan.State = 5;
            mySim.start_loop_time = mySim.t; %Restart timer
        end
    
    % State 5
    elseif (Flight_Plan.State == 5)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 15; 
        drone.u.AirSpeed = 5;
        drone.u.Xi_c = deg2rad(240);  
        
        %Trigger
        if (mySim.loop_time > 70)
            Flight_Plan.State = Flight_Plan.LANDING;
            mySim.start_loop_time = mySim.t; %Restart timer
        end
        
        
    %Coming in for a landing
    elseif (Flight_Plan.State == Flight_Plan.LANDING)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 8; 
        drone.u.AirSpeed = 5;
        drone.u.Xi_c = deg2rad(90);
        
        %The drone should be going at _____ (airspeed) up to ____
        %(altitude) and _____ (north,east,south,west)
        
        %Trigger
        if (h<9) %If __________ 
            Flight_Plan.State = Flight_Plan.FLARE;
            mySim.start_loop_time = mySim.t; %Restart timer
        end
   elseif (Flight_Plan.State == Flight_Plan.FLARE)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        prev_speed = Va;
        prev_alt = h;
        new_speed = prev_speed - 10*mySim.dt;
        if (new_speed < 1)
            new_speed = 1;
        end
        new_alt = h-20*mySim.dt;
        if (new_alt > 0)
            new_alt = 0;
        end
        
        %Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = new_speed;
        drone.u.Xi_c = deg2rad(0);
        
        %Trigger
        if (h<1) %If __________ 
            Flight_Plan.State = Flight_Plan.TOUCHDOWN;
            mySim.start_loop_time = mySim.t; %Restart timer
        end
        
    elseif (Flight_Plan.State == Flight_Plan.TOUCHDOWN)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 0; 
        drone.u.AirSpeed = 0;
        drone.u.Xi_c = deg2rad(0);
        
        %The drone should be going at _____ (airspeed) up to ____
        %(altitude) and _____ (north,east,south,west)
        
        %Trigger
        if (mySim.loop_time > 10)
            mySim.sim_flag = false;
        end
    end
    
    %% Calculate drone dynamics
        
    drone = update_drone(mySim.dt,drone);
    
    drone.data.x(:,i) = drone.s;
    drone.data.u(:,i) = [drone.u.Height,drone.u.AirSpeed,drone.u.RollAngle,drone.u.Xi_c];

    mySim.t = mySim.t+mySim.dt;
    i = i+1;
    
    if (i>=mySim.idx_end)
        mySim.sim_flag = false;
    end
end


t = 0:mySim.dt:mySim.time_end;
i = i-1;


%% plot everything
figure;
plot(drone.data.x(2,1:i),drone.data.x(1,1:i));
axis equal;


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

figure;
plot(drone.data.x(5,1:i)); hold on;
plot(drone.data.u(2,1:i),'r');
title('Drone Waypoint Following for Changes in Airspeed')
legend('UAV actual airspeed', 'Waypoint Instruction')
subtitle('This plot depicts the airspeed history during its flight')

makeMovie("example",drone,i,50);

%% Helper Functions

%Implement P controller for calculating a roll angle from a given heading
function RollAngle = calc_RollAngle(drone)
    Xi_c = drone.u.Xi_c;

    %Keep Xi_c between 0 and 360 degrees
    if Xi_c < 0
        Xi_c = Xi_c + 2*pi;
    end
    
    %Why do we have this condition? hint: what happens if we're heading
    %north-east and we want to turn north-west
    if Xi_c-drone.s(6) < -pi
        Xi_c = Xi_c + 2*pi;
    end
    if Xi_c-drone.s(6) > pi
        Xi_c = Xi_c - 2*pi;
    end
    
    RollAngle = drone.param.bX*(Xi_c-drone.s(6));

    %Why do we have this here?
    if RollAngle > drone.param.max_bank
        RollAngle = drone.param.max_bank;
    end
    if RollAngle < -drone.param.max_bank
        RollAngle = -drone.param.max_bank;
    end
end

%Initialize the Simulation
function mySim = initialize_simulation()
    %initialize everything
    mySim.t = 0;
    mySim.time_end = 3000;
    mySim.dt = 0.01;
    mySim.idx_end = mySim.time_end/mySim.dt;
    
    mySim.start_loop_time = 10000;
    mySim.loop_time = 0;
    
    mySim.sim_flag = true;
end

%Initialize a Flight Plan
function [Flight_Plan] = initialize_flight_plan()
    Flight_Plan.State = 0;
    Flight_Plan.waypoints = [300, 400;
        300, -200;
        100, -200;
        100,300];

    Flight_Plan.waypoint_state = 0;
    Flight_Plan.n_waypoints = length(Flight_Plan.waypoints);
    Flight_Plan.w0 = 0;
    Flight_Plan.w1 = 0;
    Flight_Plan.w2 = 0;
    Flight_Plan.line_orbit = 0;
    Flight_Plan.last_line = 0;
    Flight_Plan.turn_radius = 40;

    Flight_Plan.line = 1;
    Flight_Plan.orbit = 2;
    
    Flight_Plan.LANDING = 777;
    Flight_Plan.FLARE = 888;
    Flight_Plan.TOUCHDOWN = 999;
end

%Initialize drone
function drone = initialize_drone()
    %Drone structure: p_north, p_east, altitude, altitude_d, airspeed, course, roll, roll_d
    drone.s = [0,0,0,0,5,0,0,0];
    

    %u = height, AirSpeed, RollAngle
    drone.u.Height = 10;
    drone.u.AirSpeed = 5;
    drone.u.RollAngle = 0;
    drone.u.Xi_c = 0;

    

    %What do these do?
    drone.param.bX = 0.09;
    drone.param.br = 0.6;
    drone.param.bdr = 1.1;
    drone.param.bdH = 2;
    drone.param.bH = .6;
    drone.param.bVa = .5;
    drone.param.max_bank = deg2rad(45);
end

%Update drone
function drone = update_drone(dt,drone)

    pn = drone.s(1);
    pe = drone.s(2);
    h = drone.s(3);
    h_d = drone.s(4);
    Va = drone.s(5);
    Xi = drone.s(6);
    roll = drone.s(7);
    roll_d = drone.s(8);

    % Implement P control for bank angle
    drone.u.RollAngle = calc_RollAngle(drone);
    
    %update the aircraft
    pnd = Va*cos(Xi);
    ped = Va*sin(Xi);
    h_dd = drone.param.bdH*(-h_d)+drone.param.bH*(drone.u.Height-h);
    Va_d = drone.param.bVa*(drone.u.AirSpeed-Va);
    if (Va > 2)
        Xi_d = 9.81/Va*tan(roll);
    else
        Xi_d = 0;
    end
    roll_dd = drone.param.bdr*(-roll_d)+drone.param.br*(drone.u.RollAngle-roll);
    
    drone.s(1) = pn+pnd*dt;
    drone.s(2) = pe+ped*dt;
    drone.s(3) = h+h_d*dt;
    drone.s(4) = h_d+h_dd*dt;
    drone.s(5) = Va+Va_d*dt;
    Xi = Xi+Xi_d*dt;
    if (Xi > 2*pi)
        Xi = Xi-2*pi;
    elseif (Xi < -2*pi)
        Xi = Xi+2*pi;
    end
    drone.s(6) = Xi;
    drone.s(7) = roll+roll_d*dt;
    drone.s(8) = roll_d + roll_dd*dt;
    
    
end

function makeMovie(filename,drone,idx,scale)
    
    movie_length = floor(idx/scale);
    minE = min(drone.data.x(2,:))-30;
    maxE = max(drone.data.x(2,:))+30;
    minN = min(drone.data.x(1,:))-30;
    maxN = max(drone.data.x(1,:))+30;
    clear M;
    fig_width = pow2(ceil(log2(maxE-minE)));
    fig_height = pow2(ceil(log2(maxN-minN)));
    
    figure('position', [70, 70, fig_width, fig_height]);
    for i = 1:movie_length
        clf('reset');
        plot(drone.data.x(2,1:i*scale),drone.data.x(1,1:i*scale),'k','LineWidth',2);
        hold on;
        plot(drone.data.x(2,i*scale),drone.data.x(1,i*scale),'p','MarkerSize',12,'MarkerFaceColor','m');

        title("Don't forget to edit this title name");
        xlabel("including this");
        ylabel("and this");

        grid on;
        axis([minE, maxE, minN,maxN]);

        M(i) = getframe(gcf);
    end

    filename = "simulation-"+filename+".mp4";
    v = VideoWriter(filename,'MPEG-4');
    v.Quality = 95;
    v.FrameRate = 30;
    open(v);
    writeVideo(v,M);
    close(v);
    clear M;
end