%% General Information:
%{
IRB760 Simulation using the DH Apporach based on a simplified model.

Created as part of:
Malla, O. and Shanmugavel, M. (2024),
"Simplified model to study the kinematics of manipulators
with parallelogram linkages", Industrial Robot,
Vol. ahead-of-print No. ahead-of-print.
https://doi.org/10.1108/IR-01-2024-0046 

The code requires the use of Peter Corke's Robotics Toolbox for Matlab:
https://petercorke.com/toolboxes/robotics-toolbox/
%}
%% Notes:
% 1. Workspace is still not developed
% 2. Path Planning algorithms is not revised nor tested

% Link (theta, D: joint extension, a: joint offset, alpha: joint twist)
%% Robot Specs (Dimensions are in mm):

L1 = 814.5+1; %RoboDK
%L1 = 814.5;
L12 = 300;
L2 = 1280; 
L3 = 1350; 
L4 = 290;
L5 = 270;

fprintf('IRB760 : \n\n');
fprintf('Link lengths " mm ":\n %1.2f , %2.2f , %3.2f , %4.2f: \n \n', L1, L2, L3, L4);

% Joints' limits:
qmin =  [-180 , -42, -20 , -300];
qmax =  [+180 , +85, +120, +300];

% Joint Limits Radians:
theta1_min = deg2rad(qmin(1)); % minimum joint angle 1
theta1_max = deg2rad(qmax(1)); % maximum joint angle 1
theta2_min = deg2rad(qmin(2)); % minimum joint angle 2
theta2_max = deg2rad(qmax(2)); % maximum joint angle 2
theta3_min = deg2rad(qmin(3)); % minimum joint angle 3
theta3_max = deg2rad(qmax(3)); % maximum joint angle 3
theta4_min = deg2rad(qmin(4)); % minimum joint angle 4
theta4_max = deg2rad(qmax(4)); % maximum joint angle 4

% Defining robot base relative to workspace origin point "Camera":
%Tbase1 = [1 0 0 0; 0 1 0 0; 0 0 1 -L1; 0 0 0 1];
Tbase1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
fprintf('Robot 1 base transformation from workspace base point: \n');
disp(Tbase1);

% Discretizing the joint rotations:
% SampleRate = 0.1; 
% n1 = ((qmax(1)-qmin(1))/SampleRate)+1; % 180/SampelRate = 1800 points
% n2 = ((qmax(2)-qmin(2))/SampleRate)+1;
% n3 = ((qmax(3)-qmin(3))/SampleRate)+1;

% q1 = linspace(theta1_min,theta1_max,n1);
% q2 = linspace(theta2_min,theta2_max,n2);
% q3 = linspace(theta3_min,theta2_max,n3);

% DH Method with PSEUDO JOINT:
IRB760M(1)= Link([0 , L1  , L12  , -pi/2]);
IRB760M(2)= Link([0 ,  0  , L2 , 0    ]);
IRB760M(3)= Link([0 ,  0  , L3 , 0    ]);
IRB760M(4)= Link([0 ,  0  , L4 , -pi/2]);
IRB760M(5)= Link([0 ,  L5 , 0  , 0    ]);

IRB760M(1).qlim = [theta1_min,theta1_max];
IRB760M(2).offset = -pi/2;
IRB760M(2).qlim = [theta2_min,theta2_max];
IRB760M(3).offset = pi/2 - IRB760M(2).theta;
IRB760M(3).qlim = [-pi,pi/2];
IRB760M(4).qlim = [-pi/2,pi]; % pseudo-joint
IRB760M(5).qlim = [theta4_min,theta4_max];
IRB760M(5).offset = -pi;

Rob1 = SerialLink(IRB760M,'name','IRB760');

figure;
Q0 = [0,deg2rad(0),deg2rad(0),deg2rad(0),0];

Rob1.plot(Q0,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);
Rob1.teach;


%% Test Cases:
% Target_j_n = [j1,j2,j3,j4];
% Targetn = [X,Y,Z,q1,q2,q3,q4]; quaternions

Target_j_1 = [-4.80,13.03,12.67,116];
Target_1 = [2188.105962818,-183.740391365,1495.499814056,0,0.869494929,0.493941867,0];
Target_j_2 = [14.40,24.89,19.33,-60.00];
Target_2 = [2327.047321673,597.484200716,1258.706106448,0,-0.604599115,0.796529918,0];
Target_j_3 = [31.20,37.59,38.67,-124];
Target_3 = [2074.105841443,1256.123146839,715.34796505,0,0.976672278,-0.214735327,0];
Target_j_4 = [9.60,0.33,45.33,40.00];
Target_4 = [1524.814399372,257.903051125,864.346766164,0,0.262189179,0.965016494,0];
Target_j_5 = [33.60,8.80,53.33,88];
Target_5 = [1325.998928694,880.991581893,726.566018032,0,0.457097927,0.889416373,0];
Target_j_6 = [0,31.66,45.33,116];
Target_6 = [2210.867684945,0,673.875948393,0,0.848048096,0.529919264,0];
Target_j_7 = [43.20,-10.67,52.00,24.00];
Target_7 = [863.153622666,810.555203806,738.540334832,0,-0.166768747,0.985996037,0];
Target_j_8 = [-2.40,15.57,60.67,60.00];
Target_8 = [1593.595577887,-66.791444048,600.599199516,0,0.518027009,0.85536426,0];
Target_j_9 = [12.00,18.96,38.00,20.00];
Target_9 = [2024.468890774,430.314146631,923.911420615,0,0.069756474,0.99756405,0];
Target_j_10 = [26.40,7.11,47.33,-40.00];
Target_10 = [1489.833681123,739.559861964,821.999302458,0,-0.547563223,0.836764314,0];
Target_j_11 = [-36.00,18.96,56.67,-68.00];
Target_11 = [1413.934641881,-1027.283649146,627.145866897,0,-0.275637356,0.961261696,0];
Target_j_12 = [-12.00,33.35,49.33,-36.00];
Target_12 = [2125.986210891,-451.892319145,589.685472972,0,-0.207911691,0.978147601,0];
Target_j_13 = [-38.40,4.57,18.00,112];
Target_13 = [1548.451805244,-1227.287777771,1403.263528134,0,0.966823389,0.255445758,0];
Target_j_14 = [-12.00,18.11,12.00,40.00];
Target_14 = [2258.003077852,-479.953370468,1480.386779896,0,0.438371147,0.898794046,0];
Target_j_15 = [2.40,7.11,1.33,-24.00];
Target_15 = [2096.151861897,87.854793113,1783.253372234,0,-0.22835087,0.973578903,0];
Target_j_16 = [-81.60,2.03,-4.67,-44.00];
Target_16 = [289.359995841,-1959.542056108,1933.533539843,0,0.322265695,0.94664926,0];



%% Forward Solution Test:

target = Target_5;
targetj = Target_j_5;

targetj = [targetj(1),targetj(2),targetj(3)-targetj(2),-targetj(3),targetj(4)];
ForwardTest = Rob1.fkine(deg2rad(targetj));

XYZs = ForwardTest.t;
ErrorInverse = sqrt((XYZs(1)-target(1))^2+(XYZs(2)-target(2))^2+(XYZs(3)-target(3))^2);
fprintf("Error of the forward solution for Position is: %f \n", ErrorInverse);

%for orientation error:
Os = [ForwardTest.n(1:3),ForwardTest.o(1:3),ForwardTest.a(1:3)];
qreal = target(length(target)-3:length(target));
qreal_conjugate = [qreal(1), -qreal(2:4)];
qsol = rotm2quat(Os);

[OrientError1,OrientError2,OrientError3] = quat2angle(quatmultiply(qsol, qreal_conjugate),"ZYX");
fprintf("Orientation Errors as a series of transformations ZYX: %f %g %f \n",rad2deg([OrientError1,OrientError2,OrientError3]));
format longG

%targetj
XYZs
%qsol

%% Inverse Solution Test:
% Quaternions to Rotation Matrix and then Inverse solution Accuracy:

target = Target_8;
T = zeros(4,4);
T(1:3,1:3) = quat2rotm(target(length(target)-3:length(target)));
T(1:4,4) = [target(1),target(2),target(3),1]; 
%solution = S2R(Rob1.ikcon(T));
%solution = Rob1.ikcon(T);

Sol = [0, 0, 0, 0, 0];
if abs(Sol(3) - Sol(2)) < 90
   Sol = Rob1.ikcon(T);
   % Sol = IRB760Inverse(T(1:3,4),T(1:3,1:3));
end

%solution = IRB1410Inverse(T(1:3,4),T(1:3,1:3));

Tsol = Rob1.fkine(Sol);
XYZs = Tsol.t;
%for orientation error:
Os = [Tsol.n(1:3),Tsol.o(1:3),Tsol.a(1:3)];
qreal = target(length(target)-3:length(target));
qreal_conjugate = [qreal(1), -qreal(2:4)];
qsol = rotm2quat(Os);
[OrientError1,OrientError2,OrientError3] = quat2angle(quatmultiply(qsol, qreal_conjugate),"ZYX");

Sol(5) = Sol(5) + OrientError1;

Tsol = Rob1.fkine(Sol);
XYZs = Tsol.t;
%for orientation error:
Os = [Tsol.n(1:3),Tsol.o(1:3),Tsol.a(1:3)];
qreal = target(length(target)-3:length(target));
qreal_conjugate = [qreal(1), -qreal(2:4)];
qsol = rotm2quat(Os);
[OrientError1,OrientError2,OrientError3] = quat2angle(quatmultiply(qsol, qreal_conjugate),"ZYX");

ErrorInverse = sqrt((XYZs(1)-target(1))^2+(XYZs(2)-target(2))^2+(XYZs(3)-target(3))^2);
fprintf("Error of the inverse solution for Position is: %f \n", ErrorInverse);
fprintf("Orientation Errors as a series of transformations ZYX: %f %g %f \n",rad2deg([OrientError1,OrientError2,OrientError3]));

rad2deg([Sol(1),Sol(2),-Sol(4),Sol(5)])

%% Fkine:

Q = deg2rad(R2S([-165,-40,-20,-300]));
Rob1.plot(Q,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);

Rob1.fkine(Q)


%% Ikine:
% Q = deg2rad(R2S([100,50,50,0]));
% Rf = Rob1.fkine(Q);
% TP = Rf.t;

IntGuess = [0,0,0,0,0];
TP = [2263.918, 231.557, 1729.781]

% Call the inverse kinematics function (IRB760Inverse) to compute Qsol
Ik = IRB760Inverse(TP, IntGuess, true)
Rob1.fkine(Ik)
Rob1.plot(Ik);
rad2deg(Ik)
rad2deg(S2R(Ik))

%% Calculating the Workspace from the DH-Method:
% Calculating the Workspace: 

x   = 1;
% N   = 40;
N = 20;
q1 = (linspace(deg2rad(-90) ,deg2rad(90)    ,N))';
q2 = (linspace(deg2rad(0)   ,deg2rad(90)    ,N))';
q3 = (linspace(deg2rad(-10) ,deg2rad(90)    ,N))';
% For faster results q4 will be considered as 0
% q4 = (linspace(deg2rad(-90) ,deg2rad(90)    ,N))'; 
% TR1 = [zeros(4,4,N^4)];

TR1 = zeros(4,4,N^3);

for i = 1:length(q1)
    for j=1:length(q2)
        for ii = 1:length(q3)
            % for jj = 1:length(q4)  
               % TR1(:,:,x) = Rob1.fkine([q1(i),q2(j),q3(ii)-q2(j),-q3(ii),q4(jj)]);
               % for faster results q4 is considered 0 as it will not have
               % effects on the workspace with the system studied
               Q = [q1(i),q2(j),q3(ii)-q2(j),-q3(ii),0];
               TR1(:,:,x) = Rob1.fkine(Q);
               x = x+1;
            % end
        end
    end
end

A1 = transl(TR1);

% Centroid and Alpha Shape
% K-Means clsutering to find the center of mass for the points of Robot 2:
num_clusters1 = 1;
[~, centroid1] = kmeans(A1 , num_clusters1);

Shrink_Factor = 0.8;
[k_A,vol_A] = boundary(A1,Shrink_Factor);


%% Plotting the workspace of the Robot:

figure;
%Rob1.plot([0,0,0,0,0]);
%c = [0.2 0.2 0.6];
c=  [0.4 0.4 0.7];
% scatter3(A1(:,1),A1(:,2),A1(:,3),100,'filled','MarkerEdgeColor','b','MarkerFaceColor',c,'MarkerEdgeAlpha',0,'MarkerFaceAlpha',0.2)
scatter3(A1(:,1),A1(:,2),A1(:,3),'filled','CData', A1(:,2));
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of IRB760 from the DH model: ");
axis equal;
hold off;

map = [0 0 0.2; 0 0 0.3; 0 0 0.4; 0 0 0.45; 0.1 0.1 0.5; 0.1 0.1 0.55; 
    0.15 0.15 0.6; 0.2 0.2 0.65; 0.25 0.25 0.7; 0.3 0.3 0.8; 0.4 0.4 0.9];

figure;
custom_colormap = map;

min_cdata = min(A1(:,1));
max_cdata = max(A1(:,1));
normalized_cdata = (A1(:,1) - min_cdata) / (max_cdata - min_cdata);
colormap(custom_colormap);
scatter3(A1(:,1), A1(:,2), A1(:,3), 'filled', 'CData', normalized_cdata,'MarkerEdgeAlpha',0.1,'MarkerFaceAlpha',0.4);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of IRB760 from the DH model");
axis equal;
hold off;


%% Plotting the workspace of the Robot With AlphaShape:

figure;
trisurf(k_A, A1(:,1), A1(:,2), A1(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.7);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of IRB760 *from DH model* with AlphaShape: Sf =  " + (Shrink_Factor));
axis equal;
hold off;


%% Create a Path from a set of desired wayPoints
% wayPoints = [1 0 0; 0 1 0; -1 0 0; 0 -1 0; 1 0 0];
% trajectory = cscvn(wayPoints');
% hold on
% fnplt(trajectory,'r',2);

%% Creating a Circular Path:

radius = 300;
nC = 50;
angles = linspace(0, 2*pi, nC);
wayPoints.X = 200 + (radius * cos(angles));
wayPoints.Y = radius * sin(angles);
wayPoints.Z = zeros(1,nC); % Assuming the circle lies in the xy plane (z = 0)

% Plotting the circular path
plot3(wayPoints.X, wayPoints.Y, wayPoints.Z, 'r.-', 'LineWidth', 2);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Circular Path through Waypoints');


%% Test Case Path Planning:

PathPlan(Rob1,wayPoints,false);

web('Path.gif');

%% FUNCTIONS:
% 1. Symbolic Equations for Geomtric and DH approach:

syms q1v q2v q3v q4v q5v L1v L12v L2v L3v L4v L5v

% Joints' limits:
qmin =  [-180 , -42, -20 , -300];
qmax =  [+180 , +85, +120, +300];

IRB760M(1)= Link([0 , L1v  , L12v  , -pi/2]);
IRB760M(2)= Link([0 ,  0  , L2v , 0    ]);
IRB760M(3)= Link([0 ,  0  , L3v , 0    ]);
IRB760M(4)= Link([0 ,  0  , L4v , -pi/2]);
IRB760M(5)= Link([0 ,  L5v , 0  , 0    ]);

IRB760M(1).qlim = [theta1_min,theta1_max];
IRB760M(2).offset = -pi/2;
IRB760M(2).qlim = [theta2_min,theta2_max];
IRB760M(3).offset = pi/2 - IRB760M(2).theta;
IRB760M(3).qlim = [-pi,pi/2];
IRB760M(4).qlim = [-pi/2,pi]; % pseudo-joint
IRB760M(5).qlim = [theta4_min,theta4_max];
IRB760M(5).offset = -pi;

Robeq = SerialLink(IRB760M,'name','IRB760');
% Compute the end-effector pose in terms of the symbolic joint angles
Q = [q1v, q2v, q3v-q2v, -q3v, q4v];
TeqDH = Robeq.fkine(Q);
Xeq = TeqDH.t(1);
Yeq = TeqDH.t(2);
Zeq = TeqDH.t(3);
disp("The final transformation from base to EE");
display(simplify(TeqDH));

%% Jacobian
J = jacobian([Xeq;Yeq;Zeq],[q1v,q2v,q3v]);
J = subs(J, [L1v, L2v, L3v, L4v], [L1, L2, L3, L4]);
display(simplify(J));
detJ = det(J);
solutions = solve(detJ,[q1v,q2v,q3v]);
disp(simplify(detJ));
disp(solutions);

%% A Funciton to calculate the svd of iterations of the Jacobian Matrix
% across the range of the joints:
SampleRate = 50; 
n1 = ((qmax(1)-qmin(1))/SampleRate)+1; % 180/SampelRate = 1800 points
n2 = ((qmax(2)-qmin(2))/SampleRate)+1;
n3 = ((qmax(3)-qmin(3))/SampleRate)+1;

q1 = linspace(theta1_min,theta1_max,n1);
q2 = linspace(theta2_min,theta2_max,n2);
q3 = linspace(theta3_min,theta2_max,n3);

% Jn = subs(J, [q1v, q2v, q3v], [q1(i), q2(j), q3(k)]); S = svd(Jn);

for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3) 
            Jn = subs(J, [q1v, q2v, q3v], [q1(i), q2(j), q3(k)]); 
            [U, S, V] = svd(Jn);
            if diag(S) == 0
                singular_values_cell{i, j, k} = 0; %#ok<SAGROW> 
            else
                singular_values_cell{i, j, k} = 1; %#ok<SAGROW> 
            end
%             % Check for singular values close to zero tolerance = 1e-6; %
%             Adjust as needed near_singularities = find(singular_values <
%             tolerance);
        end
    end
end

%% 2. Inverse Kinematics Function:
% Call IRB760Inverse to calculate first 3 joints' variables
% If currentPose is provided, it will be used as an initial guess
% Otherwise, a default initial guess [0,42.5,52.5,0] is used
% 'interior-point' handles large, sparse problems, as well as small dense problems.
% The algorithm satisfies bounds at all iterations, and can recover from NaN or Inf results. 
% How to Use: [endEffector, currentPose] = End_effector_position;
function [Q, numIterations, timetaken] = IRB760Inverse(endEffector,currentPose,showOutput)
    % Robot Link Lengths
  
    L1 = 814.5+1;
    L12 = 300;
    L2 = 1280; 
    L3 = 1350; 
    L4 = 290;
    L5 = 270;

    % Define the objective function (error to minimize)
    % Set default value for showOutput argument
    if nargin < 4
        showOutput = true; % Display text by default
    end

    objective = @(q) norm([(cos(q(1))*(L12 + L4 + L2*sin(q(2)) + L3*cos(q(3)))) - endEffector(1);
                           (sin(q(1))*(L12 + L4 + L2*sin(q(2)) + L3*cos(q(3)))) - endEffector(2);
                           (L1 - L5 + L2*cos(q(2)) - L3*sin(q(3))) - endEffector(3)]);
    % Initial guess:     initial_guess = [0, 42.5, 52.5, 0];

    % Set initial_guess to currentPose if it's supplied, otherwise use [0, 0, 0, 0]
    if nargin < 2
        initial_guess = [0, deg2rad(40), deg2rad(50), 0];
    else
        initial_guess = S2R(currentPose);
    end

    % No constraints for now
    % Joints' limits:
    qmin =  [-180 , -42, -20 , -300];
    qmax =  [+180 , +85, +120, +300];
    
    % Define lower and upper bounds (optional, if needed)
    lb = [deg2rad(qmin(1)), deg2rad(qmin(2)), deg2rad(qmin(3)), deg2rad(qmin(4))];
    ub = [deg2rad(qmax(1)), deg2rad(qmax(2)), deg2rad(qmax(3)), deg2rad(qmax(4))];
    % Call fmincon
    % options = optimoptions('fmincon','Display', 'iter','MaxIterations', 200,'TolFun', 1e-8 ,'TolX', 1e-8);
   
    if showOutput
        tic;
        options = optimoptions('fmincon','Algorithm','sqp', 'Display', 'iter','MaxIterations', 200,'TolFun', 1e-12 ,'TolX', 1e-12);
    else
        options = optimoptions('fmincon','Algorithm','sqp', 'Display', 'off','MaxIterations', 200,'TolFun', 1e-12 ,'TolX', 1e-12);
    end

    [q_optimal, ~, ~, output] = fmincon(objective, initial_guess, [], [], [], [], lb, ub, [], options);

    if showOutput
        timetaken = toc;
    end

    % Access the number of iterations from the output structure
    numIterations = output.iterations;
    % Calculate Q values using DH parameters
    Q = [q_optimal(1), q_optimal(2), q_optimal(3) - q_optimal(2), -q_optimal(3), q_optimal(4)]; 

end


%% 3. Function to update the plot when the slider value changes:
function sliderCallback(source, ~ , A)
    [init_azimuth, init_elevation] = view;
    Shrink_Factor = source.Value;     % Gets the current value of the slider
    
    [k_A, ~] = boundary(A, Shrink_Factor); % Re-calculate and Re-plot
    [~, centroid1] = kmeans(A , 1);
    plot3(centroid1(1), centroid1(2), centroid1(3), "+", "MarkerSize", 15, "Color", "black", "LineWidth", 2);
    trisurf(k_A, A(:,1), A(:,2), A(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.5);
    view(init_azimuth, init_elevation);
    axis equal;
    hold off;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title("IRB760 Workspace with AlphaShape: Sf = " + Shrink_Factor);

    drawnow; % Re-plot
end



%% 5. Function to convert real manipulator variables into simulation:
function Qout = R2S(Qin)
    Qout = [Qin(1),Qin(2),Qin(3)-Qin(2),-Qin(3),Qin(4)];
end

%% 6. Function to convert simulation variables into real manipulator:
function qout = S2R(qin)
    qout = [qin(1),qin(2),-qin(4),qin(5)];
end

%% 7. Function to calculate and plot the path for the robot

function [Qsol, Qreal] = PathPlan(theRobot,Path,ShowOutput)
    % Plotting the entire path
    figure;
    gif('Path.gif','DelayTime',1/10,'LoopCount',1000);

    theRobot.plot([0,0,0,0,0]);
    hold on;
    plot3(Path.X, Path.Y, Path.Z, 'r.-', 'LineWidth', 2);
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Path through Waypoints');

    n = length(Path.X);
    Qsol = zeros(n, 5); % Initializing Qsol for all waypoints
    Qreal = zeros(n,4);

    for i = 1:n
        TP = [Path.X(i), Path.Y(i), Path.Z(i)];
        IntGuess = theRobot.getpos; % Starting solution from current point
        
        % Call the inverse kinematics function (IRB760Inverse) to compute Qsol
        [Qsol(i, :)] = IRB760Inverse(TP, IntGuess, ShowOutput);
    
        if ShowOutput        
            % Display the joint angles in degrees
            disp('Q values: '); 
            disp(rad2deg(Qsol(i, :)));
        end
    end
    for i = 1:n
        % Plot the robot for each waypoint
        theRobot.plot(Qsol(i, :), 'view', [40 20], 'wrist', 'jaxes', 'arrow', 'jointcolor', [0.3, 0.3, 0.3], 'linkcolor', [1, 1, 1], 'tile1color', [0.9, 0.9, 0.9], 'tile2color', [0.9, 0.9, 0.9]);
        gif;
    end

    % Plotting the path again for clarity
    plot3(Path.X, Path.Y, Path.Z, 'r.-', 'LineWidth', 2);
    hold off;
end
