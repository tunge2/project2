%clear all; close all;

%data = importdata('S_letter_path.mat');


function [v, lambda] = makenormalvectors(data)
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

