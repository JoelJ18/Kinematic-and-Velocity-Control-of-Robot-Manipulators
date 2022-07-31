clear, clc, close all
addpath('utils');

plotOn = true; 
nTests = 20;

%% Create the manipulator
mdl_stanford
robot=stanf;

qlim = [-180  180;  % q(1)
    -125  125;  % q(2)
    0  2;  % q(3)
    -270  270;  % q(4)
    -120  133.5;% q(5)
    -270  270]; % q(6)

q = zeros(1,6);
S_space=[0,0,1,0,0,0;
         0,1,0,-0.412,0,0;
         0,0,0,0,0,1;
         0,0,1,0.154,0,0;
         1,0,0,0,0.412,-0.154;
         0,0,1,0.154,0,0]';

M=[0,1,0,0;
    -1,0,0,0.154;
    0,0,1,0.675;
    0,0,0,1];


S_body=space2body(S_space,M,q)



if plotOn
   stanf.teach(zeros(1,6)); 
end

%% YOUR CODE HERE
fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 10 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the forward kinematics
    
    T = fkine(S_body,M,q,"body");
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');
%% Part B - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 10 random sets of joiny
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the Jacobian in the body frame
    J_b = jacobe(S_space,M,q);
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)]; % swap the rotation and translation components
    assert(all(all(abs(double(robot.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


% Part C - Calculate the Analyical Jacobian of the manipulator
fprintf('---------------------Analytical Jacobian Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Analytical Jacobian for 10 random sets of joint
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
        qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
        qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
        qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
        qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
        qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the Analytical Jacobian
    J_a = jacoba(S_space,M,q);
    
    if plotOn
        robot.teach(q);
        title('Analytical Jacobian Test');
    end
    
    % Test the correctness of the Jacobian
    Jref = robot.jacob0(q);
    Jref = Jref(1:3,:);
    assert(all(all(abs(double(Jref) - J_a) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Set the current joint variables
currentQ = [0,0,0,0,0,0];

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S_space,M,currentQ);

% Generate path to follow
t = linspace(0, 2*pi, nTests);
x = 1 * cos(t);
y = 1 * sin(t);
z = 1 * ones(1,nTests);
path = [x; y; z];

if plotOn
    robot.teach(currentQ);
    h = plot_ellipse(J_a*J_a');
    title('Inverse Kinematics Test');
    hold on
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end
     
% Iterate over the target points
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S_body, M, currentQ, 'body');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        % YOUR INVERSE KINEMATICS CODE HERE
        % Necessary variables:
        % Current Robot Pose -> currentPose
        % Target Robot Pose ->  targetPose
        % Current Joint Variables -> currentQ
        J = jacoba(S_space,M,currentQ);
        lambda=0.1;
        
        
   %Using Levenberg Method     
        deltaQ = J'*(pinv((J*J')+(lambda^2*eye(3))))*(targetPose - currentPose);
        currentQ = currentQ + deltaQ';
        T = fkine(S_body, M, currentQ, 'body');
        currentPose = T(1:3,4);

        if plotOn & currentQ(3)>=0
            robot.teach(currentQ);
            plot_ellipse(J*J',currentPose,'alter',h);
            title('Inverse Kinematics Test');

        end
        
    end
end

fprintf('\nTest passed successfully.\n');
