% ASSUME 3-arm inverse kinematics
% also assumes robot starts at origin
% p1 = (0, 0)
% computes a ALL robot arm orientation given arm lengths and paths

% input: T apparently
        %some set of result path points
        % arm lengths
        % L1, L2, L3
% starting point of q1 relative to q0

% output: q


%Define an object robot that has the following attributes:
%q:n×1 joint displacement vector
%P: a 3×n+ 1 link displacement vector in the zero configuration
%T:  a  4×4  homogeneous  transform  for  the  pose  (orientation  and  position)  of  the robot
%J: a 3×n matrix relating joint velocity to the end effector angular and linear velocities

%The inverse kinematics (only for a 3-dof planar arm) should take the 
%end effector pose T and return all possible solutions q

% v -> angle theta
% data -> T right corner
%

% data = importdata('S_letter_path.mat');
% robotO = [0 0]'; %where P1 is, where the robot arm 1 starts 
%                     % we have to COMPUTE the ANGLE 
%                     % q1 and q2

function [q] = tung_inversekin(data, L, robotO)

if length(L) ~= 3
    error('This aint a 3 link robot, dumbass');
end

[v, ~] = makenormalvectors(data);
n = length(data);
pathT = zeros(4,4,n);
pathT(3,3,:) = 1;
pathT(4,4,:) = 1;

angleT = atan2(v(1,:),v(2,:));
for i=1:n
    pathT(1:2,1:2,i) = rot(angleT(i));
    pathT(1:2,4,i) = data(:,i);
end

% we have pT = data
pT = data;
% we can get p3
p3 = pT + v*L(3);

%plot(pT(1,:), pT(2,:)) %THIS CHECK PASSES for p3
hold on
% quiver(pT(1,:),pT(2,:),v(1,:),v(2,:))
% plot(p3(1,:), p3(2,:),'ob')

p1 = zeros(2,n);
p1(1,:) = robotO(1);
p1(2,:) = robotO(2);

% we compute p2 maybe? not even necssessary 
% because we are actually computing q1 + q2
% p3 is pT
% know lengths L(1) and L(2)
vee = p3 - p1;

% for i=1:5
%     plot([p1(1,i) p3(1,i)], [p1(2,i) p3(2,i)],'*-r','linewidth',0.1)
% end

q1sol = zeros(2,n);
q2sol = zeros(2,n);
q3sol = zeros(2,n);

q = zeros(6,n);

%plot(data(1,:),data(2,:))

for i=1:n-1 %i think the last data point is actually too far...
    vvv = norm(vee(:,i));
    vvv_angle = atan2(vee(2,i),vee(1,i));
    
    %%%%%%%%%%%%%%%%%%% compute angles %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pink = vvv_angle;
    red = acos((L(1)^2+vvv^2-L(2)^2)/(2*vvv*L(1)));
    yellow = acos((L(1)^2+L(2)^2-vvv^2)/(2*L(1)*L(2)));
    blue = atan2(v(2,i),v(1,i));
    %%%%%%%%%%%%%%%%%%% elbow up %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q1sol(1,i) = pink + red;    
    q2sol(1,i) = -(pi-yellow);
    q3sol(1,i) = red+blue-pink-pi;
    %%%%%%%%%%%%%%%%%%% elbow down %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q1sol(2,i) = pink - red;    
    q2sol(2,i) = pi-yellow;
    q3sol(2,i) = -red+blue-pink-pi;

    %q_arm_down = [q1sol(2,i) q2sol(2,i) q3sol(2,i)];  
    %[~, ~, joints_down] = tung_forwardkin(q_arm_down, L);
    %plot([robotO(1) joints_down(1,:)], [robotO(2) joints_down(2,:)],'linewidth',2);
end
q(1,:) = q1sol(1,:);
q(2,:) = q2sol(1,:);
q(3,:) = q3sol(1,:);
q(4,:) = q1sol(2,:);
q(5,:) = q2sol(2,:);
q(6,:) = q3sol(2,:);


% A = zeros(4,n);
% A(1,:) = pathT(1,4,:);
% A(2,:) = pathT(2,4,:);
% A(3,:) = v(1,:);
% A(4,:) = v(2,:);
% quiver(A(1,:),A(2,:),A(3,:),A(4,:));
% quiver(pathT(1,4,:),pathT(2,4,:),v(1,:),v(2,:)); for some reason we get
% error here. but when we put into A theres no error.

end



function R = rot(angle)
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
end














