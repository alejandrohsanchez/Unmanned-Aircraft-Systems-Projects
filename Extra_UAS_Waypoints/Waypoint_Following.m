clc
clear;
close all;

%initialize everything
mySim = initialize_simulation();

drone = initialize_drone();

drone.data.x = zeros(8,mySim.idx_end);
drone.data.u = zeros(4,mySim.idx_end);

Flight_Plan = initialize_flight_plan();

waypoints_1 =  [300, 400;
                300, -200;
                100, -200;
                100,300];
            
waypoints_2 =  [400, 100;
                100, 100;
                50, 300;
                300,250;
                400,-300];
grid = [500,-500;
        700,-350;
        550, -100;
        500, -50;
        400, -100;
        250, -200;
        300, -400;
        500, -500];
[W,T] = generateGrid(grid,80);
            
mySim.sim_flag = true;

center = [150;150];

[W] = makePath(center);

i = 1;
while mySim.sim_flag == true
    
    %Move aircraft information from the x state vector to more readable
    %variables.  Label each variable.
    pn = drone.s(1);
    pe = drone.s(2);
    h = drone.s(3);
    Va = drone.s(4);
    Xi = drone.s(5);
    pitch = drone.s(6);
    roll = drone.s(7);
    roll_d = drone.s(8);
    
    %Default command values
    drone.u.Height = 10;  
    drone.u.AirSpeed = 5;
    drone.u.RollAngle = 0; 
	flag = 1;
    
    %% State Machine
    %At each state, calculate the state output and check to see if we
    %trigger a change to another state
    if (Flight_Plan.State == 0)
        %Output
        drone.u.Height = 18;   
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = 0;
        
        %The drone should be going at _____ (airspeed) up to ____
        %(altitude) and heading ______ (north,east,south,west)
        
        %Trigger
        if (h>17) %If __________ 
            Flight_Plan.State = 1;
            mySim.start_loop_time = mySim.t; %Start timer
            Flight_Plan = update_waypoints(Flight_Plan,waypoints_1,drone);
        end
    
    % State 1 - Follow Waypoints
    elseif (Flight_Plan.State == 1)
        % Calculations
        
        % If we've already gone to the first waypoint
        if (Flight_Plan.waypoint_target > 1)
            Flight_Plan.w0 = [Flight_Plan.waypoints(Flight_Plan.waypoint_target-1,1);...
                              Flight_Plan.waypoints(Flight_Plan.waypoint_target-1,2)];
        end
        Flight_Plan.w1 = [Flight_Plan.waypoints(Flight_Plan.waypoint_target,1);...
                          Flight_Plan.waypoints(Flight_Plan.waypoint_target,2)];
                      
        % If there is a next waypoint for us to go to
        if (Flight_Plan.waypoint_target < Flight_Plan.n_waypoints)
        Flight_Plan.w2 = [Flight_Plan.waypoints(Flight_Plan.waypoint_target+1,1);...
                          Flight_Plan.waypoints(Flight_Plan.waypoint_target+1,2)];
        else
            Flight_Plan.w2 = [0;0];
        end

        w_diff = Flight_Plan.w1-Flight_Plan.w0;
        q0 = w_diff/norm(w_diff);
        w_next = Flight_Plan.w2-Flight_Plan.w1;
        q1 = w_next/norm(w_next);
        
        %%%%% Calculations for fillet method - Change this section for
        %%%%% other methods
        
        % If we need to calculate fillet turn (only not true for last
        % waypoint)
        if (Flight_Plan.waypoint_target < Flight_Plan.n_waypoints)
            Q = acos(-q0'*q1);
        else
            Q = 1.5708;
        end
        
        if Flight_Plan.waypoint_state == 1 %we do lines
            flag = 1;
            z = Flight_Plan.w1-(Flight_Plan.turn_radius/(tan(Q/2)))*q0;
            score = ([pn;pe]-z)'*q0;
            if score >= 0
                Flight_Plan.waypoint_state = 2;
                if (Flight_Plan.waypoint_target == Flight_Plan.n_waypoints)
                    Flight_Plan.waypoint_target = Flight_Plan.waypoint_target+1;
                end
            end
        elseif Flight_Plan.waypoint_state == 2 %we do circle
            flag = 2;
            c = Flight_Plan.w1-(Flight_Plan.turn_radius/sin(Q/2))*((q0-q1)/norm(q0-q1));
            lambda = sign(q0(1)*q1(2)-q0(2)*q1(1));
            z = Flight_Plan.w1+(Flight_Plan.turn_radius/tan(Q/2))*q1;
            score = ([pn;pe]-z)'*q1;
            if score >=0
                Flight_Plan.waypoint_target = Flight_Plan.waypoint_target+1;
                Flight_Plan.waypoint_state = 1;
            end
        end
        %%%%%% End calculations for fillet method
        
        %Calculate Flight Commands 
        % Set flag = 1 for flying a line
        % Set flag = 2 for flying a circle
        % Set c = [circle north, circle east] to set circle location
        if flag == 1
            QN = Flight_Plan.w1(1)-Flight_Plan.w0(1);
            QE = Flight_Plan.w1(2)-Flight_Plan.w0(2);
            Course_d = atan2(QE,QN);
            
            %should we turn left or right
            if (Course_d-Xi) < - pi
                Course_d = Course_d + 2*pi;
            end
            if (Course_d-Xi) > pi
                Course_d = Course_d - 2*pi;
            end
           
            epy = -sin(Course_d)*(pn-Flight_Plan.w0(1))+ cos(Course_d)*(pe-Flight_Plan.w0(2));
            
            Aircraft_Course = -drone.param.Xinf*(2/pi)*atan(drone.param.k_line*epy)+Course_d;

        elseif flag == 2
            cn = c(1);
            ce = c(2);
            
            d = sqrt((pn-cn)^2+(pe-ce)^2);
            psi = atan2(pe-ce,pn-cn);
            if (psi-Xi)<-pi
                psi=psi+2*pi;
            end
            if (psi-Xi)>pi
                psi=psi-2*pi;
            end
            Aircraft_Course = psi+lambda*(pi/2+atan(drone.param.k_orbit*((d-Flight_Plan.turn_radius)/Flight_Plan.turn_radius)));
        end
        
        %Output
        
        if (Flight_Plan.complete_flag == true) %if we've already completed the circle stage, fly lower
            drone.u.Height = 20;
        else
            drone.u.Height = 50; %if it's the first time, fly higher
        end
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;
        
        
        %Trigger
        if Flight_Plan.waypoint_target>Flight_Plan.n_waypoints
            
            mySim.start_loop_time = mySim.t; %Restart timer
            if Flight_Plan.complete_flag == true %if we've already completed the circle stage, go to stage 3
                Flight_Plan.State = 3;
            else
                Flight_Plan.State = 2; %otherwise go to circle stage
                Flight_Plan.circle_center = [200,0];
            end
        end
    
    %State 2 - do 2 circles
    elseif (Flight_Plan.State ==2)
        cn = Flight_Plan.circle_center(1);
        ce = Flight_Plan.circle_center(2);        
        lambda = 1;
        
        d = sqrt((pn-cn)^2+(pe-ce)^2);
        psi = wrapTo2Pi(atan2(pe-ce,pn-cn)); %[0-360]
        if (psi-Xi)<-pi
            psi=psi+2*pi;
        end
        if (psi-Xi)>pi
            psi=psi-2*pi;
        end

        Aircraft_Course = psi+lambda*(pi/2+atan(drone.param.k_orbit*((d-Flight_Plan.loiter_radius)/Flight_Plan.loiter_radius)));
    
        %Change altitude while orbiting
        prev_alt = h;
        new_alt = h-5*mySim.dt;
        if (new_alt < 10)
            new_alt = 10;
        end
        
        drone.u.Height = new_alt;
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = Aircraft_Course;

        %Trigger
        if (Flight_Plan.start_circle_count == 0) %if we haven't started circle
            if (abs(Flight_Plan.loiter_radius-d)<5) %check if we're close enough
                Flight_Plan.start_entry_angle = psi; %remember where the circle started
                Flight_Plan.start_circle_count = 1; %set the circle count
                Flight_Plan.loop_count = 0; %start the loop count
                loop_flip = 0;
            end
        end
        
        if (Flight_Plan.start_circle_count == 1) %if we've started circling
            %Debounce circuit for counting loops
            circle_amount = wrapTo2Pi(psi-Flight_Plan.start_entry_angle); %how much have we circled
            if (loop_flip == 0)
                if circle_amount > 6.2657 %are we about to do a complete circle?
                    loop_flip = 1; %set latch
                end
            end
            if (loop_flip == 1)
                if circle_amount < 0.175 %have we gone greater than 1 circle
                    Flight_Plan.loop_count = Flight_Plan.loop_count + 1; %increment counter
                    loop_flip = 0; %unlatch
                end
            end

            %have we done two loops?
            if (Flight_Plan.loop_count > 2)
                Flight_Plan.State = 1; %lets go back to waypoint following
                mySim.start_loop_time = mySim.t; %Restart timer
                Flight_Plan = update_waypoints(Flight_Plan,W,drone); %update waypoints
                Flight_Plan.complete_flag = true; %flag to say we've already done circles
            end
        end
        
    %State 3   
    elseif (Flight_Plan.State ==3)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 30; 
        drone.u.AirSpeed = 7;
        drone.u.Xi_c = deg2rad(270);  
        
        %Trigger
        if (mySim.loop_time > 10)
            Flight_Plan.State = Flight_Plan.LANDING;
            mySim.start_loop_time = mySim.t; %Restart timer
        end
        
    %Coming in for a landing
    elseif (Flight_Plan.State == Flight_Plan.LANDING)
        %calculations
        mySim.loop_time = mySim.t - mySim.start_loop_time; %Check timer
        
        %Output
        drone.u.Height = 8; 
        drone.u.AirSpeed = 4;
        drone.u.Xi_c = deg2rad(-30);
        
        
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

%plot_3D_flight_path(drone,i,160);

figure;
plot(drone.data.x(2,1:i),drone.data.x(1,1:i));
hold on;
axis equal;
title({'Drone Path (Spiral Pattern)'})
xlabel('Drone Position East');
ylabel('Drone Position North');
%Add title and legend to this figure
plot(center(1), center(2), 'co');
plot(W(:,2),W(:,1),'rs');
legend('Drone Path', 'Center of path', 'Waypoints');
%makeMovie("example",drone,i,40);
%make3DMovie("Grid-Example",drone,i,40);


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
    Flight_Plan.waypoints = [300, 400;
        300, -200;
        100, -200;
        100,300];

    Flight_Plan.waypoint_target = 1;
    Flight_Plan.n_waypoints = length(Flight_Plan.waypoints);
    Flight_Plan.w0 = 0;
    Flight_Plan.w1 = 0;
    Flight_Plan.w2 = 0;
    Flight_Plan.line_orbit = 0;
    Flight_Plan.last_line = 0;
    Flight_Plan.turn_radius = 30;
    Flight_Plan.loiter_radius = 40;
    
    Flight_Plan.start_circle_count = 0;
    Flight_Plan.circle_center = [0;0];
    Flight_Plan.loop_count = 0;
    Flight_Plan.start_entry_angle = 0;
    Flight_Plan.complete_flag = false;

    Flight_Plan.waypoint_state = 1;
    Flight_Plan.line = 1;
    Flight_Plan.orbit = 2;
    
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
        axis equal;
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

function make3DMovie(filename,drone,idx,scale)
    
    movie_length = floor(idx/scale);
    minE = min(drone.data.x(2,:))-30;
    maxE = max(drone.data.x(2,:))+30;
    minN = min(drone.data.x(1,:))-30;
    maxN = max(drone.data.x(1,:))+30;
    clear M;
    fig_width = pow2(ceil(log2(maxE-minE)));
    fig_height = pow2(ceil(log2(maxN-minN)));
    
    figure('position', [70, 70, fig_width, fig_height]);
    
    translations = [drone.data.x(2,:)',-drone.data.x(1,:)',-drone.data.x(3,:)'];
    rotations = eul2quat([drone.data.x(5,:)'-deg2rad(90),drone.data.x(6,:)',drone.data.x(7,:)']);
    
    
    axis equal
    for i = 1:movie_length
        
        clf('reset');
        linestart = max([1 i-30]);
        plotTransforms(translations(i*scale,:),rotations(i*scale,:),'MeshFilePath','fixedwing.stl','InertialZDirection',"down",'FrameSize',10)
        hold on;
        plot3(drone.data.x(2,linestart*scale:i*scale),drone.data.x(1,linestart*scale:i*scale),drone.data.x(3,linestart*scale:i*scale),'--b');
        axis equal
        grid on;
        xlabel('East (m)');
        ylabel('North (m)');
        zlabel('Alt (m)');
        title('Do Not Submit this Video');
        
        %axis([minE, maxE, minN,maxN]);

        M(i) = getframe(gcf);
    end

    filename = "simulation-"+filename+".mp4";
    v = VideoWriter(filename,'MPEG-4');
    v.Quality = 95;
    v.FrameRate = 24;
    open(v);
    writeVideo(v,M);
    close(v);
    clear M;
end

%% Make Grid

function [waypoints,transects] = generateGrid(corners,transect)
%northing, easting
P = corners;
T = transect;

n_P = length(P); %number of corners
v = zeros(n_P-1,2); %array of vectors

%calcualte the directions from corner to corner
for i = 1:n_P-1
    v(i,:) = P(i+1,:)-P(i,:);
end

sigm = atan2(v(1,1),v(1,2)); %direction from corner 1 to corner 2
gR = [cos(sigm),-sin(sigm);  
    sin(sigm),cos(sigm)]; %Rotation Matrix

P2 = (gR*P')'; %rotated polygon
Poly_height = P2(1,1)-min(P2(:,1)); %how tall is the rotated polygon
numT = floor((Poly_height-1/2*T)/T)+1; %how many transects can we fit in polygon

left_most_point = min(P2(:,2))-T;
right_most_point = max(P2(:,2))+T;

dir = 1; %start with corner 1 to corner 2
line_height = max(P2(:,1))+1/2*T;
wpR = zeros(numT*2,2);
for line = 1:numT
    line_height = line_height-T; %current line horizontal position (y-axis/north)
    if (dir == 1)   %calculate waypoints as a pair (left, right)
        %waypoints on line    [line_north, line_east]
        wpR((line-1)*2+1,:) = [line_height,left_most_point];
        wpR((line-1)*2+2,:) = [line_height,right_most_point];
    else %calculate waypoints as a pair (right, left)
        wpR((line-1)*2+1,:) = [line_height,right_most_point];
        wpR((line-1)*2+2,:) = [line_height,left_most_point];
    end
    dir = -dir; %reverse direction
end

waypoints = (gR'*wpR')'; %Rotate everything back to original reference frame
transects = numT;
end

function V = rotate(theta)
    V = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end

function [W] = makePath(centroid)
    theta1 = deg2rad(30);
    theta2 = deg2rad(60);
    theta3 = deg2rad(90);
    theta4 = deg2rad(120);
    theta5 = deg2rad(150);
    theta6 = deg2rad(180);
    theta7 = deg2rad(210);
    theta8 = deg2rad(240);
    theta9 = deg2rad(270);
    theta10 = deg2rad(300);
    theta11 = deg2rad(330);
    theta12 = deg2rad(360);
    theta13 = deg2rad(390);
    theta14 = deg2rad(420);
    theta15 = deg2rad(450);
    theta16 = deg2rad(480);
    theta17 = deg2rad(510);
    theta18 = deg2rad(540);
    theta19 = deg2rad(570);
    theta20 = deg2rad(600);
    theta21 = deg2rad(630);
    theta22 = deg2rad(660);
    theta23 = deg2rad(690);
    theta24 = deg2rad(710);
    
    waypoint1 = centroid + [100;0];
    waypoint2 = centroid + rotate(theta1)*[100;0];
    waypoint3 = centroid + rotate(theta2)*[100;0];
    waypoint4 = centroid + rotate(theta3)*[110;0];
    waypoint5 = centroid + rotate(theta4)*[110;0];
    waypoint6 = centroid + rotate(theta5)*[150;0];
    waypoint7 = centroid + rotate(theta6)*[150;0];
    waypoint8 = centroid + rotate(theta7)*[200;0];
    waypoint9 = centroid + rotate(theta8)*[200;0];
    waypoint10 = centroid + rotate(theta9)*[220;0];
    waypoint11 = centroid + rotate(theta10)*[220;0];
    waypoint12 = centroid + rotate(theta11)*[230;0];
    waypoint13 = centroid + rotate(theta12)*[230;0];
    waypoint14 = centroid + rotate(theta13)*[240;0];
    waypoint15 = centroid + rotate(theta14)*[240;0];
    waypoint16 = centroid + rotate(theta15)*[250;0];
    waypoint17 = centroid + rotate(theta16)*[250;0];
    waypoint18 = centroid + rotate(theta17)*[260;0];
    waypoint19 = centroid + rotate(theta18)*[260;0];
    waypoint20 = centroid + rotate(theta19)*[270;0];
    waypoint21 = centroid + rotate(theta20)*[270;0];
    waypoint22 = centroid + rotate(theta21)*[280;0];
    waypoint23 = centroid + rotate(theta22)*[280;0];
    waypoint24 = centroid + rotate(theta23)*[290;0];
    waypoint25 = centroid + rotate(theta24)*[290;0];
    waypoint26 = waypoint1;
    
    W = [waypoint1, waypoint2, waypoint3, waypoint4, waypoint5, waypoint6,waypoint7, waypoint8, waypoint9, waypoint10, waypoint11, waypoint12, waypoint13, waypoint14, waypoint15, waypoint16, waypoint17, waypoint18, waypoint19, waypoint20, waypoint21, waypoint22, waypoint23, waypoint24,waypoint25, waypoint26]';
end