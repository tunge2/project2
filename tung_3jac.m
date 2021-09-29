





function [q] = tung_3jac(T,p0T)

T
p0T

L = [1.5 1.5 0.5];
data = T(1:2,4);
n = length(data);



maxiter = 1000;
q0 = [-1.63588670230047;2.36095909483614;-2.38621253915662]
q = zeros(3,n);
alpha = 1;
for i=1:n
    counter = 0;
    while counter < maxiter
        [T, ~, ~] = tung_forwardkin(q(:,i)) 
        T
        joints = T(1:2,4,1)
        JT = makeJac(L,q(:,i))'
        p0T
        term = joints-p0T
        q(:,i) = q(:,i)-alpha.*JT*(term);
    end
    q0 = q(:,i);
end

end