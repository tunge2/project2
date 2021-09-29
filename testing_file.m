% testing file

%%%%%%%%%%%%%%%%%% Forward Kinematics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% computes a SINGLE robot arm orientation given lengths and angles

% robotO = [0 0 0]'; %assumption of robot origin and angle in rad
% q = [robotO(3) 70*pi/180 25*pi/180 70*pi/180 20*pi/180 16*pi/180 43*pi/180 89*pi/180]; %initial angle relative to origin PLUS ARMS
% L = [sqrt(robotO(1)^2+robotO(2)^2) 7 17 27 7 26 15 7]; %initial distance from origin PLUS ARMS
% 
% [T, J, joints]  = tung_forwardkin(q, L);
% 
% figure(42069)
% axis square
% axis equal
% grid on
% hold on
% plot(joints(1,:), joints(2,:), 'o-r', 'linewidth',3) %plot ALL JOINTS

%%%%%%%%%%%%%%%%%% Inverse Kinematics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% computes a ALL robot arm orientation given arm lengths and paths

%data = importdata('S_letter_path.mat');
%[v] = makenormalvectors(data);
L = [1.5 1.5 0.5];
robotO = [0 0]'; %where P1 is, where the robot arm 1 starts 
                     % we have to COMPUTE the ANGLES

%x = linspace(-2,2,100);
%y = sin(x);
%data = zeros(2,100);
%data(1,:) = x;
%data(2,:) = y;

%qqqsin = tung_inversekin(data, L, robotO);

%q = tung_inversekin(data, L, robotO);

%q = q(1:3,:);
%plot3linkrobot(42069, q, L, robotO)

% %%%%%%%%%%%%%%%%%% Verification %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %clear all; close all;
% n = 30 ;
% q = zeros(3,n);
% q(:,1) = (2*rand(3,1)-1)*pi;
% 
% change = 0.5; %rad
% for i=2:n
%     q(:,i) = q(:,i-1)+(2*rand(3,1)-1)*change;
% end
% robotO = [0 0];
% 
%  
% % [T, ~, joints] = tung_forwardkin(q);
% 
% [T, ~, joints] = tung_forwardkin(q);
% plot3linkrobot(42069, q, L, robotO)
% 
% qsol=tung_3invkin(T);
% 
% qup = qsol(1:3,1);
% [a, b, jup] = tung_forwardkin(qup);
% plot([robotO(1) jup(1,:)], [robotO(2) jup(2,:)],'-*r','linewidth',2);
% 
% qdown = qsol(4:6,1);
% [c, d, jdown] = tung_forwardkin(qdown);
% plot([robotO(1) jdown(1,:)], [robotO(2) jdown(2,:)],'-*b','linewidth',2);


% %%%%%%%%%%%%%%%%%% Jacobian %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




















