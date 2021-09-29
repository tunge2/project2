
function J=makeJac(L,q)

% q = [0.1 0.2 0.3];
n = length(L);
N = length(q);
if n~=N
    error('yo, each angle per arm dumbass');
end
J = zeros(3,n);

% L1 = L(1);
% L2 = L(2);
% L3 = L(3);
% q1 = q(1);
% q2 = q(2);
% q3 = q(3);

J(1,1:n) = 1;
for i=1:n    
    A = 0;
    B = 0;
    for j=i:n;A = A+L(j)*sin(sum(q(1:j)));end
    J(2,i) = -1*A;
    for j=i:n;B = B+L(j)*cos(sum(q(1:j)));end
    J(3,i) = B;
end

% J(2,1) = -(L1*sin(q1)+L2*sin(q1+q2)+L3*sin(q1+q2+q3));
% J(2,2) = -(L2*sin(q1+q2)+L3*sin(q1+q2+q3));
% J(2,3) = (-L3*sin(q1+q2+q3));
% J(3,1) = (L1*cos(q1)+L2*cos(q1+q2)+L3*cos(q1+q2+q3));
% J(3,2) = (L2*cos(q1+q2)+L3*cos(q1+q2+q3));
% J(3,3) = L3*cos(q1+q2+q3);

end