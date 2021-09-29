% 3-arm forward kinematics
% computes a SINGLE robot arm orientation given lengths and angles

% input: vector of angular inputs?
% q = (-pi, pi)
% arm lengths
% L1, L2, L3

% some sort of d?
%              placement vectors?
% computes a SINGLE robot arm orientation given lengths and angles


%Define an object robot that has the following attributes:
%q:n×1 joint displacement vector
%P: a 3×n+ 1 link displacement vector in the zero configuration
%T:  a  4×4  homogeneous  transform  for  the  pose  (orientation  and  position)  of  the robot
%J: a 3×nmatrix relating joint velocity to the end effector angular and linear velocities


%The forward kinematics should take the robot with joint angle q and compute T and J.  

% check q is same length as p? 
% we want to start from R_01 to P_01
%                       ^^^^ comes from q_1

%starting position


function [T, J, joints] = tung_forwardkin(q) 
L = [1.5 1.5 0.5];
n = size(q,1);
nn = length(L);

N = size(q,2);
if n==nn
    disp(['lol gj. You got ', num2str(n-1), ' robot arms' ]);
else
    error(['dude, get ur shit together. q and L are not equal lengths. rn its ', num2str(n), ' and ', num2str(nn)]);
end 

T = zeros(4,4,N);
J = zeros(3,n,N);

for i=1:N
R = zeros(3,3,n);

Tmn = zeros(4,4,n);
Tmn(4,4,:) = 1;
T0n = zeros(4,4,n);
T0n(4,4,:) = 1;

angle_now = q(1);
R(:,:,1) = makeR(angle_now);

Tmn(1:3,1:3,1) = R(:,:,1);
Tmn(1:2,4,1) = L(1)*[cos(angle_now) sin(angle_now)]';   

T0n(:,:,1) = Tmn(:,:,1); %make initial T_0i
joints(:,1) = T0n(1:2,4,1); 

%note that we have already computed the starting point p1 (based off of
%starting point origin p0 and the starting angle q0

for j=2:n
    angle_now = q(j);
    R(:,:,j) = makeR(angle_now);
    
    Tmn(1:3,1:3,j) = R(:,:,j);
    Tmn(1:2,4,j) = L(j)*[cos(angle_now) sin(angle_now)]';   
    
    T0n(:,:,j) = T0n(:,:,j-1)*Tmn(:,:,j); %make T_0i
    
    joints(:,j) = T0n(1:2,4,j)'; %storing joint points
    %plot(T0n(1,4,i), T0n(2,4,i), '*r', 'linewidth',3) %plot ALL JOINTS
end % compute the other joints
%plot(joints(1,:), joints(2,:))

T_now = T0n(:,:,end);
T(:,:,i) = T_now;

if N~=1
    J_now = makeJac(L,q(:,i));
    J(:,:,i) = J_now;
end
%J: a 3×n matrix relating joint velocity to the end effector angular and linear velocities



end


end

function R = makeR(angle)
  R=[cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
  % this is for the 2D planar case
end






