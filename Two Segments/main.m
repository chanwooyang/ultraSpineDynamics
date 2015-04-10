% @Author ChanWoo Yang
% UC Berkeley
% BEST Lab
% Dynamic Tensegrity Robotics Lab
% Intelligent Robotics Group, NASA Ames Research Center
% Created 4/03/2015
% Modified 4/10/2015
% Contact ChanWoo at: chanwoo.yang@berkeley.edu
% Tensegrity Spine Dynamics: Two Stellated Tetrahedron Segments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

dt = 0.02;      % Time step
yy10 = [0 0 0.6 0 0 50 0 0 0 0 0 0];    %Initial Condition
alpha = 109.5;    % angle b/w rods [degree]
R = 1;            % Length of Rod [m]
% Distance on 2D projection
l = R*sind(alpha/2);
h = R*cosd(alpha/2);
m = 1;              % mass of node [kg]
g = 9.81;           % gravitational acc. [m/s^2]

% Fixed first segment
node1 = [0,0,0;
        -l,0,-h;
        l,0,-h;
        0,-l,h;
        0,l,h];

% Force at each node
F1 = [0 0 0];
F2 = [0 0 0];
F3 = [0 0 0];
F4 = [0 0 0];
F5 = [0 0 0];
F0 = zeros(1,3);    % No force

T = [0];
Y = yy10;
for i = 1:(11/dt)
%     if i == 1/dt
%         F2 = [0 0 0];
%     end
    options = odeset('reltol',1.e-5,'abstol',1.e-5);
    [T1,Y1] = ode45(@(t,yy)eomSolver(t,yy,node1,l,h,m,g,F1,F2,F3,F4,F5),[0,dt],yy10,options);
    yy10 = Y1(end,:);                   %Update initial condition
    T = cat(1,T,T(end,:)+T1(end,:));    %Store time step
    Y = cat(1,Y,Y1(end,:));             %Store states at each time step
end

node = getNodeCoord(T,Y);   % node([x,y,z],node#,timeStep)

for i = 1:length(T)
    figure(1)
    plot3(node(1,1,i),node(2,1,i),node(3,1,i),'*b',...
          node(1,2,i),node(2,2,i),node(3,2,i),'*r',...
          node(1,3,i),node(2,3,i),node(3,3,i),'*k',...
          node(1,4,i),node(2,4,i),node(3,4,i),'*m',...
          node(1,5,i),node(2,5,i),node(3,5,i),'*y',...
          [node(1,1,i) node(1,2,i)],[node(2,1,i) node(2,2,i)],[node(3,1,i) node(3,2,i)],'-b',...
          [node(1,1,i) node(1,3,i)],[node(2,1,i) node(2,3,i)],[node(3,1,i) node(3,3,i)],'-b',...
          [node(1,1,i) node(1,4,i)],[node(2,1,i) node(2,4,i)],[node(3,1,i) node(3,4,i)],'-b',...
          [node(1,1,i) node(1,5,i)],[node(2,1,i) node(2,5,i)],[node(3,1,i) node(3,5,i)],'-b',...
          node1(1,1),node1(1,2),node1(1,3),'*r',...
          node1(2,1),node1(2,2),node1(2,3),'*b',...
          node1(3,1),node1(3,2),node1(3,3),'*k',...
          node1(4,1),node1(4,2),node1(4,3),'*m',...
          node1(5,1),node1(5,2),node1(5,3),'*y',...
          [node1(1,1) node1(2,1)],[node1(1,2) node1(2,2)],[node1(1,3) node1(2,3)],'-r',...
          [node1(1,1) node1(3,1)],[node1(1,2) node1(3,2)],[node1(1,3) node1(3,3)],'-r',...
          [node1(1,1) node1(4,1)],[node1(1,2) node1(4,2)],[node1(1,3) node1(4,3)],'-r',...
          [node1(1,1) node1(5,1)],[node1(1,2) node1(5,2)],[node1(1,3) node1(5,3)],'-r');
    grid on
    axis equal
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
%     view([0,90])
    pause(0.1)
    T(i)
end