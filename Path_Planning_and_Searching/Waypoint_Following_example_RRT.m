clc
clear;
close all;

%Set RUN_SIM to true to run the flight simulation
RUN_SIM = true;
MOVIE = false;

%% initialize everything
mySim = initialize_simulation();

drone = initialize_drone();

drone.data.x = zeros(8,mySim.idx_end);
drone.data.u = zeros(4,mySim.idx_end);

Flight_Plan = initialize_flight_plan();

[map,NW,SE] = generateMap();

%All coordinates in North, East
entrance = [400 1];
target = [400 750];
waypoints_1 = generateRRT(map,NW,SE,entrance,target);


%Draw Map
obstacle_map = map;
im_obstacle = flipud(1-obstacle_map);
figure;
imagesc(im_obstacle)
colormap(gray)
set(gca,'YDir','normal')
hold on;
plot(waypoints_1(:,2),waypoints_1(:,1),'s--','MarkerSize',8,...
    'MarkerEdgeColor','red',...
    'MarkerFaceColor',[1 .6 .6]);
score = mean(mean(map))-0.0794;%Obstacles - border
title_str = sprintf('Map Score: %0.2f',score*100);
title(title_str);
hold off;



if RUN_SIM
mySim.sim_flag = true;

%Start Drone at entrance
drone.s(1)=entrance(1);
drone.s(2)=entrance(2)-100;
drone.s(5)=deg2rad(90);

i = 1;
while mySim.sim_flag == true
    
    if (mod(i,100)==0)
        clc;
        str = "Progress: " + i + " frame";
        disp(str);
    end
    
    %Move aircraft information from the x state vector to more readable
    %variables.  
    pn = drone.s(1);
    pe = drone.s(2);
    h = drone.s(3);

    %Default command values
    drone.u.Height = 10;
    drone.u.AirSpeed = 5;
    drone.u.RollAngle = 0;
    
    %% State Machine
    % State 0 - Launch %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (Flight_Plan.State == 0)
        %Output
        drone.u.Height = 18; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = deg2rad(90);
        
        %Trigger
        if (h>17) %If __________
            Flight_Plan.State = 1;
            mySim.start_loop_time = mySim.t; %Start timer
            Flight_Plan = update_waypoints(Flight_Plan,waypoints_1,drone);
        end
        
    % State 1 - Follow Waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan.State == 1)
        % Calculations
        
        [flag,r,q,c,rho,lambda,Flight_Plan] = followWaypoints(Flight_Plan,drone);   
        
        %Output
        drone.u.Height = 50; 
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = calculateCommand(flag,r,q,c,rho,lambda,drone);
        
        %Trigger
        if Flight_Plan.waypoint_target>Flight_Plan.n_waypoints
            
            mySim.start_loop_time = mySim.t; %Restart timer
            Flight_Plan.State = 2; 
            Flight_Plan.circle_center = [pn,pe];
        end
        
    %State 2 - do circles until landing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan.State ==2)
        flag = 2;
        c = Flight_Plan.circle_center;
        rho = Flight_Plan.loiter_radius;
        lambda = 1;
        r = 0;
        q = 0;
        
        %Output
        drone.u.Height = change_altitude(-20*mySim.dt,10,drone);
        drone.u.AirSpeed = 8;
        drone.u.Xi_c = calculateCommand(flag,r,q,c,rho,lambda,drone);
        
        %Trigger
        if (h < 12) 
            Flight_Plan.State = Flight_Plan.LANDING; 
            mySim.start_loop_time1 = mySim.t; %Restart timer
        end
        
 
    %State LANDING - Come in for a landing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan.State == Flight_Plan.LANDING)
        %calculations
        mySim.loop_time1 = mySim.t - mySim.start_loop_time1; %Check timer
        
        %Output
        drone.u.Height = 4;
        drone.u.AirSpeed = 4;
        drone.u.Xi_c = deg2rad(-30);
            
        %Trigger
        if (h<5) %If __________
            Flight_Plan.State = Flight_Plan.FLARE;
            mySim.start_loop_time1 = mySim.t; %Restart timer
        end
        
    %State FLARE - Slowdown on approach %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan.State == Flight_Plan.FLARE)
        %calculations
        mySim.loop_time1 = mySim.t - mySim.start_loop_time1; %Check timer    
        
        %Output
        drone.u.Height = change_altitude(-10*mySim.dt,0,drone);
        drone.u.AirSpeed = change_speed(-10*mySim.dt,1,drone);
        drone.u.Xi_c = deg2rad(0);
        
        %Trigger
        if (h<1) %If __________
            Flight_Plan.State = Flight_Plan.TOUCHDOWN;
            mySim.start_loop_time1 = mySim.t; %Restart timer
        end
    
    % State TOUCHDOWN - Wait 10 seconds after landing %%%%%%%%%%%%%%%%%%%%%
    elseif (Flight_Plan.State == Flight_Plan.TOUCHDOWN)
        %calculations
        mySim.loop_time1 = mySim.t - mySim.start_loop_time1; %Check timer
        
        %Output
        drone.u.Height = 0;
        drone.u.AirSpeed = 0;
        drone.u.Xi_c = deg2rad(0);
             
        %Trigger
        if (mySim.loop_time1 > 10)
            mySim.sim_flag = false;
        end
    end
    
    
    %% Calculate drone dynamics
    
    drone = update_drone(mySim.dt,drone);
    drone.data.x(:,i) = drone.s;
    drone.data.u(:,i) = [drone.u.Height,drone.u.AirSpeed,drone.u.RollAngle,drone.u.Xi_c];
    
        
    mySim.t = mySim.t+mySim.dt;
    i = i+1;
    
    %end simulation if we run out of space
    if (i>=mySim.idx_end)
        mySim.sim_flag = false;
    end
end


t = 0:mySim.dt:mySim.time_end;
i = i-1;


%% plot everything

%Draw Map
obstacle_map = map;
im_obstacle = flipud(1-obstacle_map);
figure;
imagesc(im_obstacle)
colormap(gray)
set(gca,'YDir','normal')
hold on;

plot(drone.data.x(2,1:i),drone.data.x(1,1:i));
axis equal;
title({'Flight path of simulated drone take off, RRT waypoint following, and landing'})
xlabel('Position East');
ylabel('Position North');
axis equal;

if MOVIE
    makeMovie("RRT Flight 2d",drone,i,40,im_obstacle)
end
end

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
Flight_Plan.turn_radius = 20;
Flight_Plan.loiter_radius = 20;

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

function makeMovie(filename,drone,idx,scale,map)

movie_length = floor(idx/scale);
minE = -130;
maxE = 1000+30;
minN = 0;
maxN = 800;
clear M;
fig_width = pow2(ceil(log2(maxE-minE)));
fig_height = pow2(ceil(log2(maxN-minN)));

figure('position', [70, 70, fig_width, fig_height]);
for i = 1:movie_length
    clf('reset');
    imagesc(map)
    set(gca,'YDir','normal')
    colormap(gray);
    hold on;
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

function make3DMovie2(filename,drone1,drone2,idx,scale)

movie_length = floor(idx/scale);
clear M;

figure('position', [70, 70, 720, 720]);

translations1 = [drone1.data.x(2,:)',-drone1.data.x(1,:)',-drone1.data.x(3,:)'];
rotations1 = eul2quat([drone1.data.x(5,:)'-deg2rad(90),drone1.data.x(6,:)',drone1.data.x(7,:)']);
translations2 = [drone2.data.x(2,:)',-drone2.data.x(1,:)',-drone2.data.x(3,:)'];
rotations2 = eul2quat([drone2.data.x(5,:)'-deg2rad(90),drone2.data.x(6,:)',drone2.data.x(7,:)']);

axis equal
for i = 1:movie_length
    
    clf('reset');
    linestart = max([1 i-30]);
    plotTransforms(translations1(i*scale,:),rotations1(i*scale,:),'MeshFilePath','fixedwing.stl','InertialZDirection',"down",'FrameSize',10)
    hold on;
    plot3(drone1.data.x(2,linestart*scale:i*scale),drone1.data.x(1,linestart*scale:i*scale),drone1.data.x(3,linestart*scale:i*scale),'--b');
    
    plotTransforms(translations2(i*scale,:),rotations2(i*scale,:),'MeshFilePath','fixedwing.stl','InertialZDirection',"down",'FrameSize',10)
    plot3(drone2.data.x(2,linestart*scale:i*scale),drone2.data.x(1,linestart*scale:i*scale),drone2.data.x(3,linestart*scale:i*scale),'--b');
    axis equal
    grid on;
    xlabel('East (m)');
    ylabel('North (m)');
    zlabel('Alt (m)');
    title('Do Not Submit this Video');
    
    %axis([minE, maxE, minN,maxN]);
    
    M(i) = getframe(gcf);
end

filename = "Two-simulation-"+filename+".mp4";
v = VideoWriter(filename,'MPEG-4');
v.Quality = 95;
v.FrameRate = 24;
open(v);
writeVideo(v,M);
close(v);
clear M;
end

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

function [map,NW,SE] = generateMap()

size_west = 0;
size_east = 999;
size_north = 799;
size_south = 0;

%Create translation adjustment - world to map coordinates
% map coordinates (1:x,1:y) - everything must shift right and up
x = size_east-size_west+1;
y = size_north-size_south+1;


%Create map (drawn in map coordinates)
obstacle_map = zeros(y, x);

%Draw obstacles by setting the pixels to 1
%These will draw the outer border - don't change this
obstacle_map(1:20,:) = 1; %this draws an obstacle in rows 1:20 from 1:end - ie, thick top border
obstacle_map(780:end,:) = 1; %this draws an obstacle in row end from 1:end - thick bottom border
obstacle_map([1:300, 510:800],1:20) = 1; %this draws an obstacle from top to 300, and 510:800 - left border with opening
obstacle_map([1:300, 510:800],980:end) = 1; %same but on the right side

%Draw Obstacles
%Only Change these lines
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obstacle_map(380:420,200:320) = 1; %thick horizontal line

obstacle_map(380:420,450:550) = 1;

obstacle_map(280:520,[180:210 300:330 420:450 540:570 660:690 780:810]) = 1; %middle vertical lines

obstacle_map(1:120, [180:210 300:330 420:450 540:570 660:690 780:810]) = 1; %bottom vertical lines

obstacle_map(end-120:end, [180:210 300:330 420:450 540:570 660:690 780:810]) = 1; %top vertical lines

obstacle_map(120:300, [180:210 660:690]) = 1;

obstacle_map(end-400:end, 420:450) = 1;

obstacle_map(450:490, 680:800) = 1;

obstacle_map(520:630,360:390) = 1; %Draw vertical line

obstacle_map(200:250, 350:390) = 1;

obstacle_map(100:600, 60:90) = 1;

obstacle_map(590:630, 60:380) = 1;

obstacle_map(200:240, 350:570) = 1;

obstacle_map(590:620, 550:900) = 1;

obstacle_map(300:630, 870:900) = 1;

obstacle_map(350:380, 800:900) = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%outputs
map = flipud(obstacle_map);
NW = [size_north,size_west];
SE = [size_south,size_east];
end

function waypoints = generateRRT(map,NW,SE,starting_point,ending_point)

    size_north = NW(1);
    size_west = NW(2);
    size_south = SE(1);
    size_east = SE(2);
    x = size_east-size_west+1;
    y = size_north-size_south+1;


    obstacle_map = map;
    obstacle_map = flipud(obstacle_map);
    im_obstacle = 1-obstacle_map;
    
    figure
    imagesc(im_obstacle)
    colormap(gray)
    set(gca,'YDir','normal')
    hold on;
    axis tight;
    starting_point = circshift(starting_point,1);
    ending_point = circshift(ending_point,1);
    
    plot(starting_point(1),starting_point(2),'o','MarkerSize',8,...
        'MarkerEdgeColor','green',...
        'MarkerFaceColor',[.6 1 .6]);
    plot(ending_point(1),ending_point(2),'o','MarkerSize',8,...
        'MarkerEdgeColor','red',...
        'MarkerFaceColor',[1 .6 .6]);
    
    %Give map a small buffer to avoid getting too close to walls
    se = strel('diamond',4);
    obstacle_map = imdilate(obstacle_map,se);
    
    explore_distance = 50;
    G = graph;
    G = addnode(G,{'Start'});
    G.Nodes.pos=starting_point;
    closest_point = starting_point;
    closest_point_index = 0;
    flag_complete = 0;


    nodes = 5000; %Max number of test nodes

    for k=1:nodes
        if flag_complete == 0
            test_point = [randi(x),randi(y)]; %pick random point        

            max_distance = 1000;
            existing_node_positions = G.Nodes.pos;
            existing_node_names = G.Nodes.Name;
            for h=1:length(G.Nodes.Name)
                if (norm(test_point-existing_node_positions(h,:))<max_distance)
                    max_distance = norm(test_point-existing_node_positions(h,:));
                    closest_point = existing_node_positions(h,:);
                    closest_point_index = h;
                end
            end
            vect_tp_to_cp = test_point-closest_point;
            if (norm(vect_tp_to_cp) ==0)
                hit_wall_flag = 1;
            else
                next_point = (vect_tp_to_cp/norm(vect_tp_to_cp))*explore_distance+closest_point;
                hit_wall_flag = 0;
                for i=1:explore_distance
                    tp = closest_point+i*(vect_tp_to_cp/norm(vect_tp_to_cp));
                    tp = round(tp);
                    if (tp(1)<1)
                        tp(1) = 1;
                        hit_wall_flag=1;
                    end
                    if (tp(2)<1)
                        tp(2) = 1;
                        hit_wall_flag=1;
                    end
                    if (tp(1)>x)
                        tp(1)=x;
                        hit_wall_flag=1;
                    end
                    if (tp(2)>y)
                        tp(2)=y;
                        hit_wall_flag=1;
                    end
                    if (obstacle_map(round(tp(2)),round(tp(1)))) %Hit map obstacle
                        hit_wall_flag=1;
                    end
                    
                end
            end

            if (hit_wall_flag == 0)
                node_name = char('V'+string(k));
                NodeProps = table({node_name},next_point,'VariableNames',{'Name','pos'});
                G = addnode(G,NodeProps);
                G = addedge(G,existing_node_names(closest_point_index),{node_name},norm(next_point-closest_point));
                plot([closest_point(1),next_point(1)],[closest_point(2),next_point(2)],'m--');
                plot(next_point(1),next_point(2),'gs','MarkerSize',8);
                drawnow

                if (norm(next_point-ending_point)<explore_distance)
                    flag_complete = 1;
                    NodeProps = table({'Target'},ending_point,'VariableNames',{'Name','pos'});
                    G = addnode(G,NodeProps);
                    G = addedge(G,{node_name},{'Target'},norm(next_point-ending_point));
                end
            end
        end


    end

    if (flag_complete ==1)
        P = shortestpath(G,{'Start'},{'Target'});
        waypoints_map = G.Nodes.pos(findnode(G,P),:);

        %plot on map coordinates
        for m = 2:length(waypoints_map)
            plot([waypoints_map(m-1,1),waypoints_map(m,1)],[waypoints_map(m-1,2),waypoints_map(m,2)],'b--','LineWidth',2);
        end

        %Translate to world coordinates
        waypoints_world = waypoints_map + [size_west-1,size_south-1];
        waypoints = circshift(waypoints_world,1,2);
    else
        waypoints = [0,0];
    end
end

function new_alt = change_altitude(speed,floor,drone)
    new_alt = drone.s(3)+speed;
    if (new_alt < floor)
        new_alt = floor;
    end
end

function new_speed = change_speed(speed,floor,drone)
    new_speed = drone.s(4)+speed;
    if (new_speed < floor)
        new_speed = floor;
    end
end

function [flag,r,q,c,rho,lambda,Flight_Plan] = followWaypoints(Flight_Plan,drone)
    pn = drone.s(1);
    pe = drone.s(2);
    flag = 0;
    r = 0;
    q = 0;
    c = 0;
    rho = Flight_Plan.turn_radius;
    lambda = 0;
    
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


    if (Flight_Plan.waypoint_target < Flight_Plan.n_waypoints)
        Q = acos(-q0'*q1);
    else
        Q = 1.5708;
    end

    if Flight_Plan.waypoint_state == 1 %we do lines
        flag = 1;
        r = Flight_Plan.w0;
        q = q0;
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
end

function Aircraft_Course = calculateCommand(flag,r,q,c,rho,lambda,drone)
    pn = drone.s(1);
    pe = drone.s(2);
    Xi = drone.s(5);
    %Calculate Flight Commands
    % Set flag = 1 for flying a line
    % Set flag = 2 for flying a circle
    % Set c = [circle north, circle east] to set circle location
    if flag == 1
        Course_d = atan2(q(2),q(1));

        %should we turn left or right
        if (Course_d-Xi) < - pi
            Course_d = Course_d + 2*pi;
        end
        if (Course_d-Xi) > pi
            Course_d = Course_d - 2*pi;
        end

        epy = -sin(Course_d)*(pn-r(1))+ cos(Course_d)*(pe-r(2));

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
        Aircraft_Course = psi+lambda*(pi/2+atan(drone.param.k_orbit*((d-rho)/rho)));
    end
end
