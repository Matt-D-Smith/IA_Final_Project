% Course Project Matthew Smith

% Based off the paper Machine vision and fuzzy logic-based
% navigation control of a goal-oriented
% mobile robot by Nattarith and Guzel

close all
clear all
rng(1,'twister');

% Define a map with object types of wall, obstacle, and goal
objects( 1).type = "wall";   objects( 1).points = [-8,  5;  8,  5];
objects( 2).type = "wall";   objects( 2).points = [ 8,  5;  8, -5];
objects( 3).type = "wall";   objects( 3).points = [ 8, -5; -8, -5];
objects( 4).type = "wall";   objects( 4).points = [-8, -5; -8,  5];
objects( 5).type = "wall";   objects( 5).points = [-8,  1;  -6.5,  1];
objects( 6).type = "wall";   objects( 6).points = [-5,  1;  5.5 ,  1];
objects( 7).type = "wall";   objects( 7).points = [ 7,  1;  8   ,  1];
objects( 8).type = "wall";   objects( 8).points = [-8, -1;  -6.5, -1];
objects( 9).type = "wall";   objects( 9).points = [-5, -1;  8   , -1];
objects(10).type = "wall";   objects(10).points = [-0.5, 5; -0.5, 1];
objects(11).type = "goal";   objects(11).points = [ -5.5, 3;  -5.2, 3];
objects(12).type = "wall";   objects(12).points = [-5.75, 0.75;  -5.75, 1.25];
objects(13).type = "wall";   objects(13).points = [-5.75, 1.25;  -6.5, 1.25];
objects(14).type = "wall";   objects(14).points = [-6.5, 1.25;  -6.5, 0.75];
objects(15).type = "wall";   objects(15).points = [-6.5, 0.75;  -5.75, 0.75];

% Create our robot friend
% He is a square with two wheels of diameter the size of the square side
robot.timeStep      = 0.1;     % Time step to run simulation, in seconds 
robot.updateStep    = 0.1;      % Time step to update controls of robot, in seconds
robot.pos           = [6,3];    % Map units
robot.angle         = 3*pi/2;        % radians
robot.wheelspeedL   = 0;        % rev/sec
robot.wheelspeedR   = 0;        % rev/sec
robot.size          = 0.2;      % Side length / wheel diameter of robot

% Initialize our senses
senses.d_obs        = 0;
senses.d_wpt        = 0;
senses.a_wpt_obs    = 0;
senses.a_obs        = 0;
senses.a_wpt        = 0;
senses.waypoint     = robot.pos;
senses.goalFound    = 0;
% rays for printing
numRays = 60;
senses.rayDists     = zeros(1,numRays);
senses.rayAngles    = zeros(1,numRays);

% Initial control vector
controlVec = 1 + 1i;

% Path
path = robot.pos;

% Make our Fuzzy controllers MFs
% d_wpt indices = [1,2,3] = ["Near", "Midway", "Far"]
d_wpt_params = [0 2 3.5; 2 3.5 5; 3.5 5 7];         % distance to waypoint
% d_obs indices = [1,2,3] = ["Near", "Midway", "Far"]
d_obs_params = [0 0.4 0.7; 0.4 0.7 1; 0.7 1 1.2];   % distance to obstacle
% a_obs indices = [1,2,3] = ["Small", "Moderate", "Big"]
a_obs_params = deg2rad([0 60 90; 60 90 140; 90 140 180]);    % angle between robot view direction and obstacle
% a_wpt_obs indices = [1,2,3] = ["Small", "Moderate", "Big"]
a_wpt_obs_params = deg2rad([0 30 60; 30 60 90; 60 90 180]);  % angle between waypoint and obstacle
% b_coe_o indices = [1,2,3,4,5] = ["Very Low", "Low", "Moderate", "High", "Very High"]
b_coe_o = [0 0 0.25; 0 0.25 0.5; 0.25 0.5 0.75; 0.5 0.75 1; 0.75 1 1]; % Behavior coefficient for B_0
% b_coe_s indices = [1,2,3,4,5,6,7] = ["Very Very Low", "Very Low", "Low", "Moderate", "High", "Very High", "Very Very High"]
b_coe_s = [0 0 1/6; 0 1/6 2/6; 1/6 2/6 3/6; 2/6 3/6 4/6; 3/6 4/6 5/6; 4/6 5/6 1; 5/6 1 1]; % Behavior coefficient for B_t, B_w, and B_wd

% define rules as indexes between input and output parameters
% input MFs are implied to be AND conditions, input MxN MF indices, 
% M is number of rules, N is number of input parameters to consider. 
% Only supports one output index per rule

% Go_To_Target Rule relations
R_B_t( 1).inputMFs(:)  = [1	1	1]; R_B_t( 1).outputMF = 5; % if d_obs is Near   and a_wpt_obs is Small    and d_wpt is Near   then B_t is High
R_B_t( 2).inputMFs(:)  = [1	1	2]; R_B_t( 2).outputMF = 4; % if d_obs is Near   and a_wpt_obs is Small    and d_wpt is Midway then B_t is Moderate
R_B_t( 3).inputMFs(:)  = [1	1	3]; R_B_t( 3).outputMF = 3; % if d_obs is Near   and a_wpt_obs is Small    and d_wpt is Far    then B_t is Low
R_B_t( 4).inputMFs(:)  = [1	2	1]; R_B_t( 4).outputMF = 4; % if d_obs is Near   and a_wpt_obs is Moderate and d_wpt is Near   then B_t is Moderate
R_B_t( 5).inputMFs(:)  = [1	2	2]; R_B_t( 5).outputMF = 3; % if d_obs is Near   and a_wpt_obs is Moderate and d_wpt is Midway then B_t is Low
R_B_t( 6).inputMFs(:)  = [1	2	3]; R_B_t( 6).outputMF = 2; % if d_obs is Near   and a_wpt_obs is Moderate and d_wpt is Far    then B_t is Very Low
R_B_t( 7).inputMFs(:)  = [1	3	1]; R_B_t( 7).outputMF = 3; % if d_obs is Near   and a_wpt_obs is Big      and d_wpt is Near   then B_t is Low
R_B_t( 8).inputMFs(:)  = [1	3	2]; R_B_t( 8).outputMF = 2; % if d_obs is Near   and a_wpt_obs is Big      and d_wpt is Midway then B_t is Very Low
R_B_t( 9).inputMFs(:)  = [1	3	3]; R_B_t( 9).outputMF = 1; % if d_obs is Near   and a_wpt_obs is Big      and d_wpt is Far    then B_t is Very Very Low
R_B_t(10).inputMFs(:)  = [2	1	1]; R_B_t(10).outputMF = 6; % if d_obs is Midway and a_wpt_obs is Small    and d_wpt is Near   then B_t is Very High
R_B_t(11).inputMFs(:)  = [2	1	2]; R_B_t(11).outputMF = 5; % if d_obs is Midway and a_wpt_obs is Small    and d_wpt is Midway then B_t is High
R_B_t(12).inputMFs(:)  = [2	1	3]; R_B_t(12).outputMF = 4; % if d_obs is Midway and a_wpt_obs is Small    and d_wpt is Far    then B_t is Moderate
R_B_t(13).inputMFs(:)  = [2	2	1]; R_B_t(13).outputMF = 5; % if d_obs is Midway and a_wpt_obs is Moderate and d_wpt is Near   then B_t is High
R_B_t(14).inputMFs(:)  = [2	2	2]; R_B_t(14).outputMF = 4; % if d_obs is Midway and a_wpt_obs is Moderate and d_wpt is Midway then B_t is Moderate
R_B_t(15).inputMFs(:)  = [2	2	3]; R_B_t(15).outputMF = 3; % if d_obs is Midway and a_wpt_obs is Moderate and d_wpt is Far    then B_t is Low
R_B_t(16).inputMFs(:)  = [2	3	1]; R_B_t(16).outputMF = 4; % if d_obs is Midway and a_wpt_obs is Big      and d_wpt is Near   then B_t is Moderate
R_B_t(17).inputMFs(:)  = [2	3	2]; R_B_t(17).outputMF = 3; % if d_obs is Midway and a_wpt_obs is Big      and d_wpt is Midway then B_t is Low
R_B_t(18).inputMFs(:)  = [2	3	3]; R_B_t(18).outputMF = 2; % if d_obs is Midway and a_wpt_obs is Big      and d_wpt is Far    then B_t is Very Low
R_B_t(19).inputMFs(:)  = [3	1	1]; R_B_t(19).outputMF = 7; % if d_obs is Far    and a_wpt_obs is Small    and d_wpt is Near   then B_t is Very Very High
R_B_t(20).inputMFs(:)  = [3	1	2]; R_B_t(20).outputMF = 6; % if d_obs is Far    and a_wpt_obs is Small    and d_wpt is Midway then B_t is Very High
R_B_t(21).inputMFs(:)  = [3	1	3]; R_B_t(21).outputMF = 5; % if d_obs is Far    and a_wpt_obs is Small    and d_wpt is Far    then B_t is High
R_B_t(22).inputMFs(:)  = [3	2	1]; R_B_t(22).outputMF = 6; % if d_obs is Far    and a_wpt_obs is Moderate and d_wpt is Near   then B_t is Very High
R_B_t(23).inputMFs(:)  = [3	2	2]; R_B_t(23).outputMF = 5; % if d_obs is Far    and a_wpt_obs is Moderate and d_wpt is Midway then B_t is High
R_B_t(24).inputMFs(:)  = [3	2	3]; R_B_t(24).outputMF = 4; % if d_obs is Far    and a_wpt_obs is Moderate and d_wpt is Far    then B_t is Moderate
R_B_t(25).inputMFs(:)  = [3	3	1]; R_B_t(25).outputMF = 5; % if d_obs is Far    and a_wpt_obs is Big      and d_wpt is Near   then B_t is High
R_B_t(26).inputMFs(:)  = [3	3	2]; R_B_t(26).outputMF = 4; % if d_obs is Far    and a_wpt_obs is Big      and d_wpt is Midway then B_t is Moderate
R_B_t(27).inputMFs(:)  = [3	3	3]; R_B_t(27).outputMF = 3; % if d_obs is Far    and a_wpt_obs is Big      and d_wpt is Far    then B_t is Low

% Avoid Obstacle Rule relations
R_B_o(1).inputMFs(:)  = [1 1]; R_B_o(1).outputMF = 3; % if a_obs is Small    and d_obs is Near   then B_o is Moderate
R_B_o(2).inputMFs(:)  = [1 2]; R_B_o(2).outputMF = 2; % if a_obs is Small    and d_obs is Midway then B_o is Low
R_B_o(3).inputMFs(:)  = [1 3]; R_B_o(3).outputMF = 1; % if a_obs is Small    and d_obs is Far    then B_o is Very Low
R_B_o(4).inputMFs(:)  = [2 1]; R_B_o(4).outputMF = 4; % if a_obs is Moderate and d_obs is Near   then B_o is High
R_B_o(5).inputMFs(:)  = [2 2]; R_B_o(5).outputMF = 3; % if a_obs is Moderate and d_obs is Midway then B_o is Moderate
R_B_o(6).inputMFs(:)  = [2 3]; R_B_o(6).outputMF = 2; % if a_obs is Moderate and d_obs is Far    then B_o is Low
R_B_o(7).inputMFs(:)  = [3 1]; R_B_o(7).outputMF = 5; % if a_obs is Big      and d_obs is Near   then B_o is Very High
R_B_o(8).inputMFs(:)  = [3 2]; R_B_o(8).outputMF = 4; % if a_obs is Big      and d_obs is Midway then B_o is High
R_B_o(9).inputMFs(:)  = [3 3]; R_B_o(9).outputMF = 3; % if a_obs is Big      and d_obs is Far    then B_o is Moderate

% Wall_Follow Rule relations
R_B_w( 1).inputMFs(:)  = [1	1	1]; R_B_w( 1).outputMF = 3; % if a_obs is Small    and a_wpt_obs is Small    and d_obs is Near   then B_w is Low
R_B_w( 2).inputMFs(:)  = [1	1	2]; R_B_w( 2).outputMF = 2; % if a_obs is Small    and a_wpt_obs is Small    and d_obs is Midway then B_w is Very Low
R_B_w( 3).inputMFs(:)  = [1	1	3]; R_B_w( 3).outputMF = 1; % if a_obs is Small    and a_wpt_obs is Small    and d_obs is Far    then B_w is Very Very Low
R_B_w( 4).inputMFs(:)  = [1	2	1]; R_B_w( 4).outputMF = 3; % if a_obs is Small    and a_wpt_obs is Moderate and d_obs is Near   then B_w is Low
R_B_w( 5).inputMFs(:)  = [1	2	2]; R_B_w( 5).outputMF = 3; % if a_obs is Small    and a_wpt_obs is Moderate and d_obs is Midway then B_w is Low
R_B_w( 6).inputMFs(:)  = [1	2	3]; R_B_w( 6).outputMF = 2; % if a_obs is Small    and a_wpt_obs is Moderate and d_obs is Far    then B_w is Very Low
R_B_w( 7).inputMFs(:)  = [1	3	1]; R_B_w( 7).outputMF = 4; % if a_obs is Small    and a_wpt_obs is Big      and d_obs is Near   then B_w is Moderate
R_B_w( 8).inputMFs(:)  = [1	3	2]; R_B_w( 8).outputMF = 3; % if a_obs is Small    and a_wpt_obs is Big      and d_obs is Midway then B_w is Low
R_B_w( 9).inputMFs(:)  = [1	3	3]; R_B_w( 9).outputMF = 3; % if a_obs is Small    and a_wpt_obs is Big      and d_obs is Far    then B_w is Low
R_B_w(10).inputMFs(:)  = [2	1	1]; R_B_w(10).outputMF = 3; % if a_obs is Moderate and a_wpt_obs is Small    and d_obs is Near   then B_w is Low
R_B_w(11).inputMFs(:)  = [2	1	2]; R_B_w(11).outputMF = 2; % if a_obs is Moderate and a_wpt_obs is Small    and d_obs is Midway then B_w is Very Low
R_B_w(12).inputMFs(:)  = [2	1	3]; R_B_w(12).outputMF = 1; % if a_obs is Moderate and a_wpt_obs is Small    and d_obs is Far    then B_w is Very Very Low
R_B_w(13).inputMFs(:)  = [2	2	1]; R_B_w(13).outputMF = 4; % if a_obs is Moderate and a_wpt_obs is Moderate and d_obs is Near   then B_w is Moderate
R_B_w(14).inputMFs(:)  = [2	2	2]; R_B_w(14).outputMF = 3; % if a_obs is Moderate and a_wpt_obs is Moderate and d_obs is Midway then B_w is Low
R_B_w(15).inputMFs(:)  = [2	2	3]; R_B_w(15).outputMF = 2; % if a_obs is Moderate and a_wpt_obs is Moderate and d_obs is Far    then B_w is Very Low
R_B_w(16).inputMFs(:)  = [2	3	1]; R_B_w(16).outputMF = 6; % if a_obs is Moderate and a_wpt_obs is Big      and d_obs is Near   then B_w is Very High
R_B_w(17).inputMFs(:)  = [2	3	2]; R_B_w(17).outputMF = 6; % if a_obs is Moderate and a_wpt_obs is Big      and d_obs is Midway then B_w is Very High
R_B_w(18).inputMFs(:)  = [2	3	3]; R_B_w(18).outputMF = 4; % if a_obs is Moderate and a_wpt_obs is Big      and d_obs is Far    then B_w is Moderate
R_B_w(19).inputMFs(:)  = [3	1	1]; R_B_w(19).outputMF = 5; % if a_obs is Big      and a_wpt_obs is Small    and d_obs is Near   then B_w is High
R_B_w(20).inputMFs(:)  = [3	1	2]; R_B_w(20).outputMF = 4; % if a_obs is Big      and a_wpt_obs is Small    and d_obs is Midway then B_w is Moderate
R_B_w(21).inputMFs(:)  = [3	1	3]; R_B_w(21).outputMF = 2; % if a_obs is Big      and a_wpt_obs is Small    and d_obs is Far    then B_w is Very Low
R_B_w(22).inputMFs(:)  = [3	2	1]; R_B_w(22).outputMF = 5; % if a_obs is Big      and a_wpt_obs is Moderate and d_obs is Near   then B_w is High
R_B_w(23).inputMFs(:)  = [3	2	2]; R_B_w(23).outputMF = 4; % if a_obs is Big      and a_wpt_obs is Moderate and d_obs is Midway then B_w is Moderate
R_B_w(24).inputMFs(:)  = [3	2	3]; R_B_w(24).outputMF = 3; % if a_obs is Big      and a_wpt_obs is Moderate and d_obs is Far    then B_w is Low
R_B_w(25).inputMFs(:)  = [3	3	1]; R_B_w(25).outputMF = 7; % if a_obs is Big      and a_wpt_obs is Big      and d_obs is Near   then B_w is Very Very High
R_B_w(26).inputMFs(:)  = [3	3	2]; R_B_w(26).outputMF = 7; % if a_obs is Big      and a_wpt_obs is Big      and d_obs is Midway then B_w is Very Very High
R_B_w(27).inputMFs(:)  = [3	3	3]; R_B_w(27).outputMF = 4; % if a_obs is Big      and a_wpt_obs is Big      and d_obs is Far    then B_w is Moderate

% Wander Rule relations
R_B_wd( 1).inputMFs(:)  = [1	1	1]; R_B_wd( 1).outputMF = 5; % if d_obs is Near   and a_wpt_obs is Small    and d_wpt is Near   then B_wd is High
R_B_wd( 2).inputMFs(:)  = [1	1	2]; R_B_wd( 2).outputMF = 4; % if d_obs is Near   and a_wpt_obs is Small    and d_wpt is Midway then B_wd is Moderate
R_B_wd( 3).inputMFs(:)  = [1	1	3]; R_B_wd( 3).outputMF = 3; % if d_obs is Near   and a_wpt_obs is Small    and d_wpt is Far    then B_wd is Low
R_B_wd( 4).inputMFs(:)  = [1	2	1]; R_B_wd( 4).outputMF = 6; % if d_obs is Near   and a_wpt_obs is Moderate and d_wpt is Near   then B_wd is Very High
R_B_wd( 5).inputMFs(:)  = [1	2	2]; R_B_wd( 5).outputMF = 5; % if d_obs is Near   and a_wpt_obs is Moderate and d_wpt is Midway then B_wd is High
R_B_wd( 6).inputMFs(:)  = [1	2	3]; R_B_wd( 6).outputMF = 4; % if d_obs is Near   and a_wpt_obs is Moderate and d_wpt is Far    then B_wd is Moderate
R_B_wd( 7).inputMFs(:)  = [1	3	1]; R_B_wd( 7).outputMF = 7; % if d_obs is Near   and a_wpt_obs is Big      and d_wpt is Near   then B_wd is Very Very High
R_B_wd( 8).inputMFs(:)  = [1	3	2]; R_B_wd( 8).outputMF = 6; % if d_obs is Near   and a_wpt_obs is Big      and d_wpt is Midway then B_wd is Very High
R_B_wd( 9).inputMFs(:)  = [1	3	3]; R_B_wd( 9).outputMF = 5; % if d_obs is Near   and a_wpt_obs is Big      and d_wpt is Far    then B_wd is High
R_B_wd(10).inputMFs(:)  = [2	1	1]; R_B_wd(10).outputMF = 4; % if d_obs is Midway and a_wpt_obs is Small    and d_wpt is Near   then B_wd is Moderate
R_B_wd(11).inputMFs(:)  = [2	1	2]; R_B_wd(11).outputMF = 3; % if d_obs is Midway and a_wpt_obs is Small    and d_wpt is Midway then B_wd is Low
R_B_wd(12).inputMFs(:)  = [2	1	3]; R_B_wd(12).outputMF = 2; % if d_obs is Midway and a_wpt_obs is Small    and d_wpt is Far    then B_wd is Very Low
R_B_wd(13).inputMFs(:)  = [2	2	1]; R_B_wd(13).outputMF = 5; % if d_obs is Midway and a_wpt_obs is Moderate and d_wpt is Near   then B_wd is High
R_B_wd(14).inputMFs(:)  = [2	2	2]; R_B_wd(14).outputMF = 4; % if d_obs is Midway and a_wpt_obs is Moderate and d_wpt is Midway then B_wd is Moderate
R_B_wd(15).inputMFs(:)  = [2	2	3]; R_B_wd(15).outputMF = 3; % if d_obs is Midway and a_wpt_obs is Moderate and d_wpt is Far    then B_wd is Low
R_B_wd(16).inputMFs(:)  = [2	3	1]; R_B_wd(16).outputMF = 6; % if d_obs is Midway and a_wpt_obs is Big      and d_wpt is Near   then B_wd is Very High
R_B_wd(17).inputMFs(:)  = [2	3	2]; R_B_wd(17).outputMF = 5; % if d_obs is Midway and a_wpt_obs is Big      and d_wpt is Midway then B_wd is High
R_B_wd(18).inputMFs(:)  = [2	3	3]; R_B_wd(18).outputMF = 4; % if d_obs is Midway and a_wpt_obs is Big      and d_wpt is Far    then B_wd is Moderate
R_B_wd(19).inputMFs(:)  = [3	1	1]; R_B_wd(19).outputMF = 3; % if d_obs is Far    and a_wpt_obs is Small    and d_wpt is Near   then B_wd is Low
R_B_wd(20).inputMFs(:)  = [3	1	2]; R_B_wd(20).outputMF = 2; % if d_obs is Far    and a_wpt_obs is Small    and d_wpt is Midway then B_wd is Very Low
R_B_wd(21).inputMFs(:)  = [3	1	3]; R_B_wd(21).outputMF = 1; % if d_obs is Far    and a_wpt_obs is Small    and d_wpt is Far    then B_wd is Very Very Low
R_B_wd(22).inputMFs(:)  = [3	2	1]; R_B_wd(22).outputMF = 4; % if d_obs is Far    and a_wpt_obs is Moderate and d_wpt is Near   then B_wd is Moderate
R_B_wd(23).inputMFs(:)  = [3	2	2]; R_B_wd(23).outputMF = 3; % if d_obs is Far    and a_wpt_obs is Moderate and d_wpt is Midway then B_wd is Low
R_B_wd(24).inputMFs(:)  = [3	2	3]; R_B_wd(24).outputMF = 2; % if d_obs is Far    and a_wpt_obs is Moderate and d_wpt is Far    then B_wd is Very Low
R_B_wd(25).inputMFs(:)  = [3	3	1]; R_B_wd(25).outputMF = 5; % if d_obs is Far    and a_wpt_obs is Big      and d_wpt is Near   then B_wd is High
R_B_wd(26).inputMFs(:)  = [3	3	2]; R_B_wd(26).outputMF = 4; % if d_obs is Far    and a_wpt_obs is Big      and d_wpt is Midway then B_wd is Moderate
R_B_wd(27).inputMFs(:)  = [3	3	3]; R_B_wd(27).outputMF = 3; % if d_obs is Far    and a_wpt_obs is Big      and d_wpt is Far    then B_wd is Low

% Run main loop of controller
% Run the controller update for the robot at a slower time step than the
% movement update so simulate dsicrete updates in continuous time

% Draw room
initializePlot();

maxTime = 100;
time = 0;
fNum = 0; % Frame Number
v = VideoWriter("ProjectRecording5","MPEG-4");
v.FrameRate = 10;
open(v);

while (~checkCollision(objects, robot)) && (time < maxTime)
    time = time + robot.timeStep;
    fNum = fNum + 1;

    senses = updateSenses(objects, robot, senses, controlVec);

    % Update Behavior Motor schema vectors from sensor inputs
    [F_t,F_o,F_w,F_wd] = updateBehaviors(senses);

    % Update Behavior weights
    % Go_To_Target
    B_t(fNum) = triFuzzyController([senses.d_obs senses.a_wpt_obs senses.d_wpt], [{d_obs_params}, {a_wpt_obs_params}, {d_wpt_params}], b_coe_s, R_B_t);
    % Avoid Obstacle
    B_o(fNum) = triFuzzyController([senses.a_obs senses.d_obs], [{a_obs_params}, {d_obs_params}], b_coe_o, R_B_o);
    % Wall Follow
    B_w(fNum) = triFuzzyController([senses.a_obs senses.a_wpt_obs senses.d_obs], [{a_obs_params}, {a_wpt_obs_params}, {d_obs_params}], b_coe_s, R_B_w);
    % Wander
    B_wd(fNum) = triFuzzyController([senses.a_obs senses.a_wpt_obs senses.d_obs], [{a_obs_params}, {a_wpt_obs_params}, {d_obs_params}], b_coe_s, R_B_wd);
    
    % Multiply each of our behavior vectors by our behavior weights, and sum
    % for final control vector
    controlVec = F_t * B_t(fNum) + F_o * B_o(fNum) + F_w * B_w(fNum) + F_wd * B_wd(fNum);
    
    % Use desired control vector to update the wheel motor speeds
    robot = updateMotors(robot,controlVec);

    drawMap(objects);
    drawRobot(objects,robot, controlVec, senses, path);
    drawVectors(robot,controlVec,senses,F_t,F_o,F_w,F_wd,B_t,B_o,B_w,B_wd)
    pause(0.01);

    robot = updateMovement(robot);
    path = [path; robot.pos];
    writeVideo(v,getframe(gcf));

end

close(v);



function initializePlot()
    figure("Name","Map","Position",[0,0,1600,900],'WindowState','maximized');
    annotation('rectangle',...
    [0.717015625 0.364 0.094703125 0.584]);
    
    annotation('textbox',...
    [0.717796875 0.950570342205323 0.0935312500000002 0.0190114068441068],...
    'String',{'Control Module Vectors'},...+
    'HorizontalAlignment','center',...
    'FontSize',12,...
    'FitBoxToText','off',...
    'EdgeColor',[0.941176470588235 0.941176470588235 0.941176470588235]);
    
    annotation('rectangle',...
    [0.817796875 0.364 0.094703125 0.584]);
    
    annotation('textbox',...
    [0.818578125000001 0.951330798479084 0.0935312500000002 0.0190114068441068],...
    'String','Behavior Vectors',...
    'HorizontalAlignment','center',...
    'FontSize',12,...
    'FitBoxToText','off',...
    'EdgeColor',[0.941176470588235 0.941176470588235 0.941176470588235]);
    
    annotation('rectangle',...
    [0.11896875 0.0920152091254753 0.79196875 0.136121673003801]);
    
    annotation('textbox',...
    [0.467796875000002 0.0699619771863072 0.0935312500000002 0.0190114068441067],...
    'String','Behavior Weights Over Time',...
    'HorizontalAlignment','center',...
    'FontSize',12,...
    'FitBoxToText','off',...
    'EdgeColor',[0.941176470588235 0.941176470588235 0.941176470588235]);
end

% Function to draw our room
function drawMap(objects) % need to add our robot and the path it takes on previous steps
    subplot(6, 8, [1,38])
    hold on
    cla
    title("Map")
    for objNum = 1:length(objects)
        if     objects(objNum).type == "wall", color = "Black"; 
        elseif objects(objNum).type == "obstacle", color = "Red"; 
        elseif objects(objNum).type == "goal", color = "Green"; 
        end
        line(objects(objNum).points(:,1), objects(objNum).points(:,2),"LineWidth",2,"Color",color);
    end
    xlim([-8,8]);
    ylim([-5 5]);
end

% Draw our robot and the visibility lines
function drawRobot(objects, robot, controlVec, senses, path)
    subplot(6, 8, [1,38])
    hold on
    
    % Draw path
    line(path(:,1),path(:,2),"Color","#9e9e9e");

    % This will be the same function to detect depth as the
    % updateMovement function
    numRays = length(senses.rayDists);
    
    % make the kernel show the calculated length of rays 
    for rayNum = 1:numRays
        kernel(:,:,rayNum) = [robot.pos ; robot.pos + senses.rayDists(rayNum) * [cos(robot.angle+senses.rayAngles(rayNum)), sin(robot.angle+senses.rayAngles(rayNum))]];
        line(kernel(:,1,rayNum), kernel(:,2,rayNum),"LineWidth",0.5,"Color","#03cafc");
    end

    % Draw the outline of the robot
    robotOutline = [ ... % get the outline of the robot
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 1*pi/4), sin(robot.angle + 1*pi/4)]; ...
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 3*pi/4), sin(robot.angle + 3*pi/4)]; ...
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 5*pi/4), sin(robot.angle + 5*pi/4)]; ...
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 7*pi/4), sin(robot.angle + 7*pi/4)]; ...
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 1*pi/4), sin(robot.angle + 1*pi/4)]; ...
        ];
    line(robotOutline(:,1), robotOutline(:,2),"LineWidth",1,"Color","Magenta");

    % Draw our control vector
    vec = controlVec*exp(1i*robot.angle);
    quiver(robot.pos(1),robot.pos(2),abs(vec)*cos(angle(vec)),abs(vec)*sin(angle(vec)),0);

    % Draw our waypoint
    plot(senses.waypoint(1),senses.waypoint(2),'o');
        
end

function drawVectors(robot,controlVec,senses,F_t,F_o,F_w,F_wd,B_t,B_o,B_w,B_wd)
    subplot(6, 8, 7)
    cla
    hold on; grid on; xlim([-2,2]); ylim([-2,2]);
    title("Go To Target Control")
    vec = F_t*exp(1i*robot.angle);
    line([0, abs(vec)*cos(angle(vec))],[0, abs(vec)*sin(angle(vec))],'linewidth',2);

    subplot(6, 8, 15)
    cla
    hold on; grid on; xlim([-1,1]); ylim([-1,1]);
    title("Avoid Obstacle Control")
    vec = F_o*exp(1i*robot.angle);
    line([0, abs(vec)*cos(angle(vec))],[0, abs(vec)*sin(angle(vec))],'linewidth',2);

    subplot(6, 8, 23)
    cla
    hold on; grid on; xlim([-0.5,0.5]); ylim([-0.5,0.5]);
    title("Wall Follow Control")
    vec = F_w*exp(1i*robot.angle);
    line([0, abs(vec)*cos(angle(vec))],[0, abs(vec)*sin(angle(vec))],'linewidth',2);

    subplot(6, 8, 31)
    cla
    hold on; grid on; xlim([-0.5,0.5]); ylim([-0.5,0.5]);
    title("Wander Control")
    vec = F_wd*exp(1i*robot.angle);
    line([0, abs(vec)*cos(angle(vec))],[0, abs(vec)*sin(angle(vec))],'linewidth',2);

    subplot(6, 8, 8)
    cla
    hold on; grid on; xlim([-2,2]); ylim([-2,2]);
    title("Go To Target Behavior")
    vec = F_t*exp(1i*robot.angle)*B_t(end);
    line([0, abs(vec)*cos(angle(vec))],[0, abs(vec)*sin(angle(vec))],'linewidth',2);

    subplot(6, 8, 16)
    cla
    hold on; grid on; xlim([-1,1]); ylim([-1,1]);
    title("Avoid Obstacle Behavior")
    vec = F_o*exp(1i*robot.angle)*B_o(end);
    line([0, abs(vec)*cos(angle(vec))],[0, abs(vec)*sin(angle(vec))],'linewidth',2);

    subplot(6, 8, 24)
    cla
    hold on; grid on; xlim([-0.5,0.5]); ylim([-0.5,0.5]);
    title("Wall Follow Behavior")
    vec = F_w*exp(1i*robot.angle)*B_w(end);
    line([0, abs(vec)*cos(angle(vec))],[0, abs(vec)*sin(angle(vec))],'linewidth',2);

    subplot(6, 8, 32)
    cla
    hold on; grid on; xlim([-0.5,0.5]); ylim([-0.5,0.5]);
    title("Wander Behavior")
    vec = F_wd*exp(1i*robot.angle)*B_wd(end);
    line([0, abs(vec)*cos(angle(vec))],[0, abs(vec)*sin(angle(vec))],'linewidth',2);

    subplot(6, 8, [41 42])
    cla
    hold on; grid on; ylim([0, 1]);
    title("Go To Target Behavior Weight")
    plot(B_t);
    
    subplot(6, 8, [43 44])
    cla
    hold on; grid on; ylim([0, 1]);
    title("Avoid Obstacle Behavior Weight")
    plot(B_o);

    subplot(6, 8, [45 46])
    cla
    hold on; grid on; ylim([0, 1]);
    title("Wall Follow Behavior Weight")
    plot(B_w);

    subplot(6, 8, [47 48])
    cla
    hold on; grid on; ylim([0, 1]);
    title("Wander Behavior Weight")
    plot(B_wd);

    delete(findall(gcf,'Tag','behavData'))
    annotation('textbox',...
    [0.7171875 0.247148288973384 0.0941406249999999 0.105703422053232],...
    'String',{"B_t = "+B_t(end),"B_o = "+B_o(end),"B_w = "+B_w(end),"B_wd = "+B_wd(end)},...
    'FontSize',12,...
    'FitBoxToText','off',...
    'Tag','behavData');
    %text(-7.9,-4,"B_t="+B_t(fNum)+newline+"B_o="+B_o(fNum)+newline+"B_w="+B_w(fNum)+newline+"B_w_d="+B_wd(fNum));
    
end

% Update the robots "sensors", its navigation parameters used as MF inputs
function sensesNew = updateSenses(objects, robot, senses, controlVec)
    hold on
    numRays = length(senses.rayDists);
    rayMaxDist = 20; % some distance longer than the maximum extent of the map
    wptThres = 1; % Distance to waypoint to say we are close enough to update waypoint position
    for ii = 1:length(objects)
        goalRay(ii).idx = []; % If we find an object of type goal, go to that ray index
        goalRay(ii).dist = []; % If we find an object of type goal, go to that ray index
    end

    angles = linspace(0,2*pi-2*pi/numRays, numRays); % get the angle of each ray we want to cast
    for rayNum = 1:length(angles) % make a kernel of line segments to check intersection with
        kernel(:,:,rayNum) = [robot.pos ; robot.pos + rayMaxDist * [cos(robot.angle+angles(rayNum)), sin(robot.angle+angles(rayNum))]];
    end
    rayDist(1:numRays) = rayMaxDist;
    goalDist(1:numRays) = rayMaxDist;
    for objNum = 1:length(objects) % Find the distance to each object for every ray
        for rayNum = 1:length(angles)
            [hitx,hity] = polyxpoly( ...
                kernel(:,1,rayNum), ... % x start and end coordinates
                kernel(:,2,rayNum), ... % y start and end coordinates
                objects(objNum).points(:,1), ... % check for each object
                objects(objNum).points(:,2) ...
            );
            dist = pdist([robot.pos; hitx, hity], "euclidean");
            if ~isempty(hitx)  % If there is a hit, check if the distance is less than any other object distance
                %rayDist(rayNum) = min(dist, rayDist(rayNum));
                if objects(objNum).type == "goal" % When a ray hits a goal object
                    goalDist(rayNum) = min(dist, goalDist(rayNum));
                    goalRay(objNum).idx = [goalRay(objNum).idx, rayNum];
                    goalRay(objNum).dist = [goalRay(objNum).dist, goalDist(rayNum)];
                else % Only count ray distance to obstacles / walls, not goals
                    rayDist(rayNum) = min(dist, rayDist(rayNum));
                end
            end
        end
    end

    % Determine waypoint to navigate to
    % check distance to current waypoint
    sensesNew.waypoint = senses.waypoint;

    % check to see if a ray hit a goal
    sensesNew.goalFound = senses.goalFound;
    if (sensesNew.goalFound == 0) % check the first ray that intersects a target object, do not update if a goal has been found
        for objNum = 1:length(objects)
            if objects(objNum).type == "goal" && (sensesNew.goalFound == 0) % only take the first goal found checked
                for goalRayNum = 1:length(goalRay(objNum).idx)
                    if goalRay(objNum).dist(goalRayNum) <= rayDist(goalRay(objNum).idx(goalRayNum)) % if our ray intersection with the target is the shortest intersection, we have found our target
                        sensesNew.goalFound = 1; % this is a flag our goal is found as well as the ray index when found
                        line(kernel(:,1,goalRay(objNum).idx(goalRayNum)), kernel(:,2,goalRay(objNum).idx(goalRayNum)),"LineWidth",2,"Color","Green"); % print line to our waypoint for testing
                        sensesNew.waypoint = robot.pos + goalDist(goalRay(objNum).idx(goalRayNum)) * [cos(robot.angle+angles(goalRay(objNum).idx(goalRayNum))), sin(robot.angle+angles(goalRay(objNum).idx(goalRayNum)))]; % set our waypoint on found goal
                        plot(sensesNew.waypoint(1),sensesNew.waypoint(2),'o');
                    end
                end
            end
        end
    end

    % Avoid a trap condition where we found the target but got stuck in a
    % local minima, such as missing a door, allow re-searching for target
    if (abs(controlVec) < robot.size)
        sensesNew.goalFound = 0;
    end

    % If we are not close to our target, use waypoint choosing algorithm.
    % If we are close to our waypoint, update our waypoint to determine where next to go
    % Alternately, if our last control vector magnitude, we are likely to
    % be stuck in a local minima, acquire a new target
    if sensesNew.goalFound == 0
        wptDist = pdist([robot.pos; senses.waypoint], "euclidean");
        if (wptDist < wptThres) || (abs(controlVec) < robot.size)
            % Try to target doors, obstacle edges, or lesser certain regions.
            % Areas where there is a large jump in adjacent distance values are
            % likely to be a door, scaled by the distance, as there will be more
            % spread in distant rays. Also scale by cosine of the angle in
            % order to favor objectives in the forward direction of the robot
            [~,dooridx] = max(abs((rayDist - circshift(rayDist,1)) ./ sqrt(rayDist)) .* sqrt(cos(angles)/2+0.5));
            if dooridx == 1
                if rayDist(numRays) > rayDist(dooridx) % check to see which of the two adjacent rays are longer
                    dooridx = numRays;
                end
            else
                if rayDist(dooridx - 1) > rayDist(dooridx) % check to see which of the two adjacent rays are longer
                    dooridx = dooridx - 1;
                end
            end
            line(kernel(:,1,dooridx), kernel(:,2,dooridx),"LineWidth",2,"Color","Red"); % print line to our waypoint for testing
            sensesNew.waypoint = robot.pos + (rayDist(dooridx) - 5*robot.size) * [cos(robot.angle+angles(dooridx)), sin(robot.angle+angles(dooridx))]; % Make our waypoint 2 * robot size from ray end for leeway to not crash into wall
        end
        plot(sensesNew.waypoint(1),sensesNew.waypoint(2),'o');
    end

    % Update waypoint distance
    sensesNew.d_wpt = pdist([robot.pos; senses.waypoint], "euclidean");

    % Update waypoint angle
    sensesNew.a_wpt = atan2(sensesNew.waypoint(2) - robot.pos(2),sensesNew.waypoint(1) - robot.pos(1)) - robot.angle;

    % Update closest obstacle distance and angle
    [sensesNew.d_obs, obsidx] = min(rayDist);
    sensesNew.a_obs = angles(obsidx); % angle distances are already relative to robot.angle

    % Update the angle between the waypoint and the nearest obstacle
    sensesNew.a_wpt_obs    = sensesNew.a_wpt - sensesNew.a_obs;

    % save ray distances and angles for display purposes
    sensesNew.rayDists = rayDist;
    sensesNew.rayAngles = angles;

end

function [F_t,F_o,F_w,F_wd] = updateBehaviors(senses)
    % Input is sense magnitudes and angles
    % Output is behavior vectors for each behavior
    %   F_t:  Go to target vector
    %   F_o:  Avoid obstacle vector
    %   F_w:  Wall follow vector
    %   F_wd: Wander vector
    % All behavior vectors are complex numbers representing mag, angle

    % Go to target
    % Always go to target at rate 2
    [x,y] = pol2cart(senses.a_wpt, 2);
    F_t = x+1i*y;

    % Avoid obstable
    % Always move at rate 1
    % Angle is away from the nearest obstacle
    % Move away at inverse of distance to more strongly avoid walls as the
    % robot is closer
    [x,y] = pol2cart(senses.a_obs + deg2rad(180), 0.25/senses.d_obs^2);
    % [x,y] = pol2cart(senses.a_obs + deg2rad(180), 2);
    F_o = x+1i*y;

    % Follow Wall
    % Always move at rate 0.5
    % Angle is parallel to wall, aka normal to the nearest obstacle angle
    % We also want it to always be in the forward direction, so limit angle
    % to +- 90 degrees
    if senses.a_obs <= deg2rad(180)
        a_wf = senses.a_obs - deg2rad(90);
    else
        a_wf = senses.a_obs + deg2rad(90);
    end
    [x,y] = pol2cart(a_wf, 0.5);
    F_w = x+1i*y;

    % Wander
    % Wander contributes at rate 0.1
    % Angle is random angle from forward angle in range -90 to +90
    
    %[x,y] = pol2cart(senses.a_wpt, 1); % for debug just make it go to waypoint
    [x,y] = pol2cart(rand()*pi-pi/2,0.5);
    F_wd = x+1i*y;
end

function robotNew = updateMotors(robot,controlVec)
    % Inputs are our robot and the desired control vector as an imaginary
    % For now make it instantly change to our desired vector
    % Later update to add a motor slew rate - real motors can't instantly
    % change speed

    % Speed Scaler
    speedScale = 0.2;

    % Copy all values of robot before updating
    robotNew = robot;

    % We want the magnitude of the control vector to correspond to speed
    % when magnitude is low, but we want magnitude 
    % control vector gives a sharper turn
    % Angle is angle (obviously)
    % Need to find arc length of each wheel

    % d = 1/mag(controlvec)
    % 1/d = r*angle
    % r = 1/mag/angle
    % distanceL = (r-robot.size/2)*angle
    % distanceR = (r+robot.size/2)*angle
    % distanceL = (1/mag-robot.size/2)*angle
    % from movement update formula
    % distanceL = pi * robot.size * robot.wheelspeedL * robot.timeStep;
    % pi * robot.size * robot.wheelspeedL * robot.timeStep = (1/mag/angle-robot.size/2)*angle
    % robotNew.wheelspeedL = (1/mag/angle-robot.size/2)*angle / (pi * robot.size * robot.timeStep)
    robotNew.wheelspeedL = speedScale*(min(abs(controlVec),1/abs(controlVec))/angle(controlVec)-robot.size/2)*angle(controlVec) / (pi * robot.size * robot.timeStep);
    robotNew.wheelspeedR = speedScale*(min(abs(controlVec),1/abs(controlVec))/angle(controlVec)+robot.size/2)*angle(controlVec) / (pi * robot.size * robot.timeStep);

end

% movement update function
function robotNew = updateMovement(robot)
    robotNew = robot; % Copy all parameters initially
    if robot.wheelspeedL == robot.wheelspeedR % simple case where both wheels move at the same speed, it moves forward the circumference of the wheel rotated in that time
        distance = pi * robot.size * robot.wheelspeedL * robot.timeStep; % d = pi*d*omega*dt
        robotNew.pos = robot.pos + [cos(robot.angle)*distance , sin(robot.angle)*distance];
    else % if the wheels are at different speeds, the robot will travel in an arc
        distanceL = pi * robot.size * robot.wheelspeedL * robot.timeStep; % d = pi*d*omega*dt
        distanceR = pi * robot.size * robot.wheelspeedR * robot.timeStep; % d = pi*d*omega*dt
        % Find the center point to rotate around and the angle of rotation arc length = r * theta, dL = r * theta, dR = r+size * theta, solve for theta
        theta = (distanceR - distanceL) / robot.size; % angle of rotation, if positive, counter-clockwise, if positive, clockwise
        r = distanceL / theta + robot.size/2; % distance from center of robot to origin point
        robotNew.angle = mod(robot.angle + theta, 2*pi);
        rotPoint = robot.pos + r * [cos(robot.angle + pi/2), sin(robot.angle + pi/2)]; % point to rotate our position around
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)]; % Rotation matrix
        robotNew.pos = (R*(robot.pos - rotPoint)')' + rotPoint; % Rotate about the rotation point
    end
end

% Check collision with objects
function hit = checkCollision(objects, robot)
    hit = false;
    robotOutline = [ ... % get the outline of the robot
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 1*pi/4), sin(robot.angle + 1*pi/4)]; ...
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 3*pi/4), sin(robot.angle + 3*pi/4)]; ...
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 5*pi/4), sin(robot.angle + 5*pi/4)]; ...
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 7*pi/4), sin(robot.angle + 7*pi/4)]; ...
        robot.pos + sqrt(2) * robot.size/2 * [cos(robot.angle + 1*pi/4), sin(robot.angle + 1*pi/4)]; ...
        ];
    for objNum = 1:length(objects)
        intersect = polyxpoly(robotOutline(:,1), robotOutline(:,2), objects(objNum).points(:,1), objects(objNum).points(:,2)); % polyxpoly from Mapping toolbox because checking intersections is hard
        if ~isempty(intersect) % if anything hits the box, return true
            hit = true;
            return
        end
    end
end