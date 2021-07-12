clc
clear;
close all;

%% Part 2
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
    %variables.  
    pn = drone.s(1);
    pe = drone.s(2);
    h = drone.s(3);
    Va = drone.s(4);
    Xi = drone.s(5);
    pitch = drone.s(6);
    roll = drone.s(7);
    roll_d = drone.s(8);
    
    %Default command values
    drone.u.Height = 10; % 
    drone.u.AirSpeed = 5;
    drone.u.RollAngle = 0; 
    
    %% State Machine
    % At each state, calculate the state output and check to see if we
    % trigger a change to another state
    if (Flight_Plan.State == 0)
        % Output
        drone.u.Height = 18;  
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = 0;
        
        
        % Trigger
        if (h>17) %If __________ 
            Flight_Plan.State = 6;
            mySim.start_loop_time = mySim.t; % Start timer
        end
    
  % State 1:Line Following   
    elseif (Flight_Plan.State == 6)
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        % Calculations
        
        % Add in additional line following states
        Aircraft_Course = deg2rad(30);
        
        % Add in equations to slowly change the aircraft's altitude over
        % time
        new_alt = 40;
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = 10;
        drone.u.Xi_c = Aircraft_Course;
        
        
        % Trigger
        if mySim.loop_time > 50 % What does this do?
            Flight_Plan.State = 7;
            mySim.start_loop_time = mySim.t; % Start timer
        end
        
  % State 2: Line Following   
    elseif (Flight_Plan.State == 7)
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        % Calculations
        
        % Add in additional line following states
        Aircraft_Course = deg2rad(60);
        
        % Add in equations to slowly change the aircraft's altitude over
        % time
        new_alt = 120;
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;
        
        
        % Trigger
        if mySim.loop_time > 50 % What does this do?
            Flight_Plan.State = 8;
            mySim.start_loop_time = mySim.t; % Start timer
        end
        
  % State 3:Line Following   
    elseif (Flight_Plan.State == 8)
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        % Calculations
        
        % Add in additional line following states
        Aircraft_Course = deg2rad(90);
        
        % Add in equations to slowly change the aircraft's altitude over
        % time
        new_alt = -150;
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;
        
        
        % Trigger
        if mySim.loop_time > 50 % What does this do?
            Flight_Plan.State = 9;
            mySim.start_loop_time = mySim.t; % Start timer
        end      
        
  % State 4: Line Following   
    elseif (Flight_Plan.State == 9)
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        % Calculations
        
        % Add in additional line following states
        Aircraft_Course = deg2rad(130);
        
        % Add in equations to slowly change the aircraft's altitude over
        % time
        new_alt = 250;
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;
        
        
        % Trigger
        if mySim.loop_time > 50 % What does this do?
            Flight_Plan.State = 10;
            mySim.start_loop_time = mySim.t; % Start timer
        end      
               
  % State 5: Line Following   
    elseif (Flight_Plan.State == 10)
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        % Calculations
        
        % Add in additional line following states
        Aircraft_Course = deg2rad(225);
        
        % Add in equations to slowly change the aircraft's altitude over
        % time
        new_alt = 150;
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;
        
        
        % Trigger
        if mySim.loop_time > 50 % What does this do?
            Flight_Plan.State = 11;
            mySim.start_loop_time = mySim.t; % Start timer
        end      
    
  % State 6: Line Following   
    elseif (Flight_Plan.State == 11)
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        % Calculations
        
        % Add in additional line following states
        Aircraft_Course = deg2rad(270);
        
        % Add in equations to slowly change the aircraft's altitude over
        % time
        new_alt = 150;
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;
        
        
        % Trigger
        if mySim.loop_time > 50 % What does this do?
            Flight_Plan.State = 12;
            mySim.start_loop_time = mySim.t; % Start timer
        end      
 
   % State 7: Line Following   
    elseif (Flight_Plan.State == 12)
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        % Calculations
        
        % Add in additional line following states
        Aircraft_Course = deg2rad(-45);
        
        % Add in equations to slowly change the aircraft's altitude over
        % time
        new_alt = 150;
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;
        
        
        % Trigger
        if mySim.loop_time > 50 % What does this do?
            Flight_Plan.State = 13;
            mySim.start_loop_time = mySim.t; % Start timer
        end      
 
    % State 8: Line Following   
    elseif (Flight_Plan.State == 13)
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        % Calculations
        
        % Add in additional line following states
        Aircraft_Course = deg2rad(0);
        
        % Add in equations to slowly change the aircraft's altitude over
        % time
        new_alt = 250;
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;
        
        
        % Trigger
        if mySim.loop_time > 50 % What does this do?
            Flight_Plan.State = Flight_Plan.LANDING;
            mySim.start_loop_time = mySim.t; % Start timer
        end      
        
    % Coming in for a landing
    elseif (Flight_Plan.State == Flight_Plan.LANDING)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; % Check timer
        
        % Output
        drone.u.Height = 8; 
        drone.u.AirSpeed = 4;
        drone.u.Xi_c = deg2rad(180);
   
        
        % Trigger
        if (h<9) %
            Flight_Plan.State = Flight_Plan.FLARE;
            mySim.start_loop_time = mySim.t; % Restart timer
        end
   elseif (Flight_Plan.State == Flight_Plan.FLARE)
        % calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; % Check timer
        
        % Output
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
        
        % Output
        drone.u.Height = new_alt; 
        drone.u.AirSpeed = new_speed;
        drone.u.Xi_c = deg2rad(0);
        
        % Trigger
        if (h<1) %
            Flight_Plan.State = Flight_Plan.TOUCHDOWN;
            mySim.start_loop_time = mySim.t; % Restart timer
        end
        
    elseif (Flight_Plan.State == Flight_Plan.TOUCHDOWN)
        % calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; % Check timer
        
        % Output
        drone.u.Height = 0; 
        drone.u.AirSpeed = 0;
        drone.u.Xi_c = deg2rad(0);
        
        
        % Trigger
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
%Add title and legend to this figure

plot_3D_flight_path(drone,i,160); %pass drone, length of sim, # of outlines in plot

makeMovie("example",drone,i,50);


%% Helper Functions

%Implement P controller for calculating a roll angle from a given heading
function RollAngle = calc_RollAngle(drone)
    Xi = drone.s(5);
    Xi_c = drone.u.Xi_c;

    %Keep Xi_c between 0 and 360 degrees
    Xi_c = wrapTo2Pi(Xi_c);
    
    %Why do we have this condition? hint: what happens if we're heading
    %north-east and we want to turn north-west
    if Xi_c-Xi < -pi
        Xi_c = Xi_c + 2*pi;
    end
    if Xi_c-Xi > pi
        Xi_c = Xi_c - 2*pi;
    end
    
    RollAngle = drone.param.bX*(Xi_c-Xi);

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
    
    Flight_Plan.LANDING = 777;
    Flight_Plan.FLARE = 888;
    Flight_Plan.TOUCHDOWN = 999;
end

%Initialize drone
function drone = initialize_drone()

    drone.airframe = fixedwing;
    drone.airframe.Configuration.FlightPathAngleLimits = [-.35,0.35];


    %Drone structure: p_north, p_east, altitude, airspeed, course, pitch, roll, roll_d
    drone.s = state(drone.airframe);
    drone.s(4) = 8;
    drone.u = control(drone.airframe);
    drone.e = environment(drone.airframe);

    %u = height, AirSpeed, RollAngle
    drone.u.Height = 10;
    drone.u.AirSpeed = 5;
    drone.u.RollAngle = 0;
    drone.u.Xi_c = 0;

    %Parameters
    drone.param.bX = 0.5;
    drone.param.max_bank = deg2rad(45);
    drone.param.k_orbit = 3;
    drone.param.k_line = 0.1;
    drone.param.Xinf = deg2rad(70);
end

%Update drone
function drone = update_drone(dt,drone)


    % Implement P control for bank angle
    drone.u.RollAngle = calc_RollAngle(drone);
    
    %update the aircraft
    sdot = derivative(drone.airframe,drone.s,drone.u,drone.e);
    drone.s = drone.s+sdot*dt;
    drone.s(5) = wrapTo2Pi(drone.s(5));
    
    
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
        axis equal
        hold on;
        plot(drone.data.x(2,i*scale),drone.data.x(1,i*scale),'p','MarkerSize',12,'MarkerFaceColor','m');

        title("Heptagon Simulation");
        xlabel("East Position");
        ylabel("North Position");

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

function plot_3D_flight_path(drone,i,count)
    ds_count = floor(i/count);

    downsample = 1:ds_count:i;
    translations = [drone.data.x(2,downsample)',-drone.data.x(1,downsample)',-drone.data.x(3,downsample)'];
    rotations = eul2quat([drone.data.x(5,downsample)'-deg2rad(90),drone.data.x(6,downsample)',drone.data.x(7,downsample)']);
    
    figure;
    plotTransforms(translations,rotations,'MeshFilePath','fixedwing.stl','InertialZDirection',"down",'FrameSize',10)
    hold on;
    plot3(drone.data.x(2,1:i),drone.data.x(1,1:i),drone.data.x(3,1:i),'--b');
    axis equal
    grid on;
    xlabel('East (m)');
    ylabel('North (m)');
    zlabel('Alt (m)');
end

%Update Waypoints
function [Flight_Plan] = update_waypoints(Flight_Plan,waypoints,drone)
    Flight_Plan.waypoints = waypoints;
    Flight_Plan.n_waypoints = length(Flight_Plan.waypoints);
    Flight_Plan.waypoint_target = 1;
    Flight_Plan.w0 = [drone.s(1);drone.s(2)];
    Flight_Plan.flag = 1;
    Flight_Plan.waypoint_state = 1;

end