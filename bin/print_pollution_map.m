clear;
T = load('pollution_map.mat');
[X,Y] = meshgrid(1:2001, 1:2001);
s = mesh(Y,X,T.arr);
xlabel('x')
ylabel('y')
zlabel('Concentration')
xlim([0,2001])
ylim([0,2001])
% title('Payload map')
colorbar('southoutside')

hold on 
[X,Y] = meshgrid(1:2001,1:2001);
Z = 0.0*sin(X) + 0.0*cos(Y);
C = 0.0*X.*Y;
surf(X,Y,Z,C)


