
function [] = plot3linkrobot(fignum, q, L, robotO)

figure(fignum)
hold on
grid on
axis square
axis equal
L = L + 1;
N = size(q,2);

for i=1:N-1
    [~, ~, jointsnow]  = tung_forwardkin(q(:,i));
    plot([robotO(1) jointsnow(1,:)], [robotO(2) jointsnow(2,:)]);
    pause(0.05)
end