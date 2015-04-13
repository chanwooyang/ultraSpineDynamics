% @Author ChanWoo Yang
% UC Berkeley
% BEST Lab
% Dynamic Tensegrity Robotics Lab
% Intelligent Robotics Group, NASA Ames Research Center
% Created 4/03/2015
% Modified 4/12/2015
% Contact ChanWoo at: chanwoo.yang@berkeley.edu
% Tensegrity Spine Dynamics: Two Stellated Tetrahedron Segments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

dt = 0.02;      % Time step [sec]
Tf = 20;        % Final Time Step [sec]
yy10 = [0 0 0.75 40 50 60 0 0 0 0 0 0];    % Initial Condition
                                         % close to Equlibrium at [0 0 0.75 0 0 0 0 0 0 0 0 0]
alpha = 109.5;    % angle b/w rods [degree]
R = 1;            % Length of Rod [m]
% Distance on 2D projection
l = R*sind(alpha/2);
h = R*cosd(alpha/2);
m = 1;              % mass of node [kg]
g = 9.81;           % gravitational acc. [m/s^2]

ks = 1200;          % Saddle cable spring constant [N/m]
kv = 1200;          % Vertical cable spring constant [N/m]
Ls0 = 0.5;            % Saddle cable initial cable length [m] 
Lv0 = 0.5;            % Vertical cable initial cable length [m]

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
for i = 1:(Tf/dt)
    options = odeset('reltol',1.e-10,'abstol',1.e-10);
    [T1,Y1] = ode45(@(t,yy)eomSolver(t,yy,node1,l,h,m,g,kv,ks,Ls0,Lv0,F1,F2,F3,F4,F5),[0,dt],yy10,options);
    yy10 = Y1(end,:);                   %Update initial condition
    T = cat(1,T,T(end,:)+T1(end,:));    %Store time step
    Y = cat(1,Y,Y1(end,:));             %Store states at each time step
end

node = getNodeCoord(T,Y);   % node([x,y,z],node#,timeStep)

% Plot Animation
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
    axis([-1 1 -1 1 -1 2])
    pause(0.1)
    T(i)
end

%% Plot

% Plot total energy over time step
[totalE,ke,pe] = energyConsvChecker(node1,m,l,h,kv,ks,Ls0,Lv0,T,Y);
figure()
plot(T(:),totalE(:),'k',T(:),ke(:),'r',T(:),pe(:),'b')
legend('Total Energy','Kinetic Energy','Potential Energy','Location','best')
grid on

% Plot changes in x,y,z coordinate of the center node
figure()
plot(T(:),Y(:,1),'r',T(:),Y(:,2),'b',T(:),Y(:,3),'k')
legend('x1','y1','z1','Location','best')
grid on

% Plot changes in theta,phi,psi of the structure
figure()
plot(T(:),Y(:,4),'r',T(:),Y(:,5),'b',T(:),Y(:,6),'k')
legend('theta','phi','psi','Location','best')
grid on
