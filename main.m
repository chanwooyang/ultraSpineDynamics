% @Author ChanWoo Yang
% UC Berkeley
% BEST Lab
% Dynamic Tensegrity Robotics Lab
% Intelligent Robotics Group, NASA Ames Research Center
% Created 4/03/2015
% Modified 4/04/2015
% Contact ChanWoo at: chanwoo.yang@berkeley.edu
% Tensegrity Spine Dynamics: Stellated Tetrahedron Segment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

tspan = [0,50];
yy0 = [0 0 0 0 0 0 0 0 0 0 0 0];    %Initial Condition
alpha = 109.5;    % angle b/w rods [degree]
R = 1;            % Length of Rod [m]
% Distance on 2D projection
l = R*sind(alpha/2);
h = R*cosd(alpha/2);
m = 1;              % mass of node [kg]

% Force at each node
F1 = [0 0 0];
F2 = [50 0 0];
F3 = [0 0 0];
F4 = [0 0 0];
F5 = [0 0 0];

% options = odeset('reltol',1.e-12,'abstol',1.e-12);
[T,Y] = ode45(@(t,yy)eomSolver(t,yy,l,h,m,F1,F2,F3,F4,F5),tspan,yy0);

node = getNodeCoord(T,Y);   % node([x,y,z],node#,timeStep)

for i = 1:length(T)
    figure(1)
    plot3(node(1,1,i),node(2,1,i),node(3,1,i),'*b',...
          node(1,2,i),node(2,2,i),node(3,2,i),'*r',...
          node(1,3,i),node(2,3,i),node(3,3,i),'*k',...
          node(1,4,i),node(2,4,i),node(3,4,i),'*m',...
          node(1,5,i),node(2,5,i),node(3,5,i),'*y',...
          [node(1,1,i) node(1,2,i)],[node(2,1,i) node(2,2,i)],[node(3,1,i) node(3,2,i)],'--b',...
          [node(1,1,i) node(1,3,i)],[node(2,1,i) node(2,3,i)],[node(3,1,i) node(3,3,i)],'--b',...
          [node(1,1,i) node(1,4,i)],[node(2,1,i) node(2,4,i)],[node(3,1,i) node(3,4,i)],'--b',...
          [node(1,1,i) node(1,5,i)],[node(2,1,i) node(2,5,i)],[node(3,1,i) node(3,5,i)],'--b');
    grid on
    axis equal
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
%     view([0,0])
    pause(0.2)
end