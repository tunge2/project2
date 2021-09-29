%tung_rigidbody
clear all; close all;
L = [1.5 1.5 0.5];

% define unit vectors
%
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

%
% load the letter S as a curve
%

data = importdata('S_letter_path.mat');
%figure(9999)
%plot(data(1,:),data(2,:),'linewidth',1)
hold on
grid on

data(1,:) = data(1,:)-0.02;
%plot(data(1,:),data(2,:),'linewidth',1)

hold off

% figure(1);plot(data(1,:),data(2,:),data(1,1),data(2,1),'o','linewidth',2);
% xlabel('x');ylabel('y');
% axis([-3 3 -3 3]);
% axis('square');
% axis equal
% grid;
%
[xT,yT]=setR0T(data); % <<<< you need to provide this

% robot parameters
robot.P = [zz L(1)*ex L(2)*ex L(3)*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];

% zero configuration 

robot.q=[0;0;0];
% radius of the link (as cylinders)
radius = .01;
robot_rb=defineRobot(robot,radius);
figure(10);
hold on;
show(robot_rb,robot.q,'Collision','on');
axis('square');
axis equal
xlabel('x-axis');ylabel('y-axis');
title('Planar RRR arm in zero configuration (q_1=q_2=q_3=0)')
view(0,90);
xlim([0 4]);
ylim([-1 1]);
hold off

%
n=length(data);
qsol1=zeros(3,n-1);
qsol2=zeros(3,n-1);
%[v] = makenormalvectors(data);

for i=1:n-1
    robot.T(1:3,1:4)=[xT(:,i) yT(:,i) ez [data(:,i);0]];
    robot.T(4,1:4)=[0 0 0 1];
    tstart(i)=tic;
    
    qsol=tung_3invkin(robot.T);
    % qsol = tung_3jac(robot.T, data(:,i));
    
    telapsed(i)=toc(tstart(i));
    qsol1(:,i)=qsol(1:3,1);
    qsol2(:,i)=qsol(4:6,1);    
end

figure(2);
plot(data(1,:),data(2,:),data(1,1),data(2,1),'o','linewidth',2);
hold on
grid on
grid minor
set(gca,'XMinorTick','on','YMinorTick','on')
axis([-1,3,-2,2]);axis('square');
title('Elbow Up');
hold off
for i=1:1:n-1
    robot.q=qsol1(:,i);pause(.005);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
end
fprintf('max joint speed for elbow UP: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsol1')')')');

dL_components = abs(diff(data'))';
numsteps = length(dL_components);
dL = zeros(1,numsteps);
lambda = zeros(1,numsteps);
for i=1:numsteps-1
    dL(1,i) = norm(dL_components(:,i));
    lambda(1,i+1) = lambda(1,i)+dL(1,i);
end

%%%% MAX ANGULAR VEL %%%%
dT_max = [0.1 0.1 0.1];


dT1 = abs(diff(qsol1')');
time_total_up = max(max(dT1')'./dT_max') * lambda(end);
fprintf('For elbow UP, the travel time will be: %5.4f seconds \n',time_total_up);

figure(11)
plot(lambda(1:end-1),dT1(1,:),'linewidth',2);
hold on
plot(lambda(1:end-1),dT1(2,:),'linewidth',2);
plot(lambda(1:end-1),dT1(3,:),'linewidth',2);
grid on
title('Angular Changes - Elbow Up');
ylabel('dTheta (rad)');
xlabel('lambda (m)');
legend('q1','q2','q3','location','best')
hold off

time1 = linspace(0,time_total_up,length(dT1));
dT1dt = dT1./max(max(dT1')'./dT_max');

figure(12)
plot(time1,dT1dt(1,:),'linewidth',2);
hold on
plot(time1,dT1dt(2,:),'linewidth',2);
plot(time1,dT1dt(3,:),'linewidth',2);
grid on
title('Angular Velocities - Elbow Up');
ylabel('Omega (rad/s)');
xlabel('Time (s)');
legend('q1','q2','q3','location','best')
hold off

figure(3);plot(data(1,:),data(2,:),data(1,1),data(2,1),'o','linewidth',2);
hold on
grid on
grid minor
set(gca,'XMinorTick','on','YMinorTick','on')
axis([-1,3,-2,2]);axis('square');
title('Elbow Down');
for i=1:1:n-1
    robot.q=qsol2(:,i);pause(.01);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
end
grid on
grid minor
hold off
fprintf('max joint speed for elbow DOWN: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsol2')')')');





dT2 = abs(diff(qsol2')');
time_total_down = max(max(dT2')'./dT_max') * lambda(end);
fprintf('For elbow DOWN, the travel time will be: %5.4f seconds \n',time_total_down);


figure(21)
plot(lambda(1:end-1),dT2(1,:),'linewidth',2);
hold on
plot(lambda(1:end-1),dT2(2,:),'linewidth',2);
plot(lambda(1:end-1),dT2(3,:),'linewidth',2);
grid on
title('Angular Changes - Elbow Down');
ylabel('dTheta (rad)');
xlabel('lambda (m)');
legend('q1','q2','q3','location','best')
hold off


time2 = linspace(0,time_total_down,length(dT2));
dT2dt = dT2./max(max(dT2')'./dT_max');

figure(22)
plot(time2,dT2dt(1,:),'linewidth',2);
hold on
plot(time2,dT2dt(2,:),'linewidth',2);
plot(time2,dT2dt(3,:),'linewidth',2);
grid on
title('Angular Velocities - Elbow Down');
ylabel('Omega (rad/s)');
xlabel('Time (s)');
legend('q1','q2','q3','location','best')
hold off

%************** functions *******************
%
% 3x3 skew-symmetric matrix
%
function vhat=crossmat(v)

    vhat = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];

end

function [xT, yT]=setR0T(data)
	n = length(data);
	xT = zeros(3,n);
	yT = zeros(3,n);

	for i=1:n
		[v] = makenormalvectors(data);
		v = -v;
		angle = atan2(v(1,i),v(2,i));
		xT(1:3,i) = [cos(angle) -sin(angle) 0];
		yT(1:3,i) = [sin(angle) cos(angle) 0];
	end

end

function [v] = makenormalvectors(data)
sdiff = diff(data')';
numpts = length(sdiff);

%plot(data(1,:),data(2,:),'b.')
hold on

grid on
% n = 3;
% xlim([-n n])
% ylim([-n n])
xlabel('x')
ylabel('y')
axis square
axis equal

a = [0 -1 0; 1 0 0; 0 0 0];
b = zeros(3,numpts);
b(1:2,:) = sdiff;
c = a*b;

%quiver(data(1,:),data(2,:),c(1,:),c(2,:))
lambda = zeros(1,numpts);

c = c(1:2,:); %get rid of z

for i=1:numpts
    lambda(i+1) = lambda(i) + norm(sdiff(:,i));
    c(:,i) = c(:,i)/norm(c(:,i));
end
c(:,end+1) = c(:,end); % Assume the last point
                       % has same normal vec
v = c;

end
