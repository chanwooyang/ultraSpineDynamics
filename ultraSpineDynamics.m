clear all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 6 states per node
% [ x y z theta phi psi ]
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% !! methane structure
alpha = 109.5;    %angle b/w rods
L = 5;         %length of rod
% Distance on 2D projection
l = L*sind(alpha/2);
h = L*cosd(alpha/2);

%Distance from the center of a regular tetrahedron to a node
R = norm([l/2 h/2 0]');

t2i = [-l 0 -h]';
t3i = [l 0 -h]';
t4i = [0 -l h]';
t5i = [0 l h]';

%Normalize these so we can multiply by R
t2 = t2i/norm(t2i);
t3 = t3i/norm(t3i);
t4 = t4i/norm(t4i);
t5 = t5i/norm(t5i);
% ---------------------------------
% Segment 1 (Fixed) ----------------------
node1 = [0 0 0]';
node2 = node1 + R*t2;
node3 = node1 + R*t3;
node4 = node1 + R*t4;
node5 = node1 + R*t5;
% -----------------------------------

% Segment 2 --------------------
% Rotational Angle
theta = 0; %about x-axis
phi = 30;   %about y-axis
psi = 15;  %about z-axis

% Rotational Matrix
Tx = [1 0 0;
      0 cosd(theta) sind(theta);
      0 -sind(theta) cosd(theta)];
Ty = [cosd(phi) 0 sind(phi);
      0 1 0;
      -sind(phi) 0 cosd(phi)];
Tz = [cosd(psi) sind(psi) 0;
      -sind(psi) cosd(psi) 0;
      0 0 1];
  
% Nodal Position after Rotation
e2 = Tx*Ty*Tz*t2;
e3 = Tx*Ty*Tz*t3;
e4 = Tx*Ty*Tz*t4;
e5 = Tx*Ty*Tz*t5;

node1f = [1 1 3]';
node2f = node1f + R*e2;
node3f = node1f + R*e3;
node4f = node1f + R*e4;
node5f = node1f + R*e5;
%------------------------------------------
% Segment 3 -------------------------------
% Rotational Angle
theta3 = 0; %about x-axis
phi3 = 0;   %about y-axis
psi3 = 30;  %about z-axis

% Rotational Matrix
Tx3 = [1 0 0;
      0 cosd(theta3) sind(theta3);
      0 -sind(theta3) cosd(theta3)];
Ty3 = [cosd(phi3) 0 sind(phi3);
      0 1 0;
      -sind(phi3) 0 cosd(phi3)];
Tz3 = [cosd(psi3) sind(psi3) 0;
      -sind(psi3) cosd(psi3) 0;
      0 0 1];

e2ff = Tx3*Ty3*Tz3*t2;
e3ff = Tx3*Ty3*Tz3*t3;
e4ff = Tx3*Ty3*Tz3*t4;
e5ff = Tx3*Ty3*Tz3*t5;
  
node1ff = [2 2 6]';
node2ff = node1ff + R*e2ff;
node3ff = node1ff + R*e3ff;
node4ff = node1ff + R*e4ff;
node5ff = node1ff + R*e5ff;
  
% plotting ----------------------------------------------------------------
plot3(node1(1),node1(2),node1(3),'*b',...
      node2(1),node2(2),node2(3),'*b',...
      node3(1),node3(2),node3(3),'*b',...
      node4(1),node4(2),node4(3),'*b',...
      node5(1),node5(2),node5(3),'*b',...
      [node1(1) node2(1)],[node1(2) node2(2)],[node1(3) node2(3)],'--b',...
      [node1(1) node3(1)],[node1(2) node3(2)],[node1(3) node3(3)],'--b',...
      [node1(1) node4(1)],[node1(2) node4(2)],[node1(3) node4(3)],'--b',...
      [node1(1) node5(1)],[node1(2) node5(2)],[node1(3) node5(3)],'--b',...
      node1f(1),node1f(2),node1f(3),'*r',...
      node2f(1),node2f(2),node2f(3),'*r',...
      node3f(1),node3f(2),node3f(3),'*r',...
      node4f(1),node4f(2),node4f(3),'*r',...
      node5f(1),node5f(2),node5f(3),'*r',...
      [node1f(1) node2f(1)],[node1f(2) node2f(2)],[node1f(3) node2f(3)],'--r',...
      [node1f(1) node3f(1)],[node1f(2) node3f(2)],[node1f(3) node3f(3)],'--r',...
      [node1f(1) node4f(1)],[node1f(2) node4f(2)],[node1f(3) node4f(3)],'--r',...
      [node1f(1) node5f(1)],[node1f(2) node5f(2)],[node1f(3) node5f(3)],'--r',...
      node1ff(1),node1ff(2),node1ff(3),'*m',...
      node2ff(1),node2ff(2),node2ff(3),'*m',...
      node3ff(1),node3ff(2),node3ff(3),'*m',...
      node4ff(1),node4ff(2),node4ff(3),'*m',...
      node5ff(1),node5ff(2),node5ff(3),'*m',...
      [node1ff(1) node2ff(1)],[node1ff(2) node2ff(2)],[node1ff(3) node2ff(3)],'--m',...
      [node1ff(1) node3ff(1)],[node1ff(2) node3ff(2)],[node1ff(3) node3ff(3)],'--m',...
      [node1ff(1) node4ff(1)],[node1ff(2) node4ff(2)],[node1ff(3) node4ff(3)],'--m',...
      [node1ff(1) node5ff(1)],[node1ff(2) node5ff(2)],[node1ff(3) node5ff(3)],'--m',...
      [-1 1],[0 0],[0 0],'-g',...
      [0 0],[-1 1],[0 0],'-g',...
      [0 0],[0 0],[-1 1],'-g');
grid on
axis([-10 10 -10 10 -5 10])
xlabel('X')
ylabel('Y')
zlabel('Z')

% lengths check -----------------------------------------------------------
norm(node2-node1,2)-norm(node2f-node1f,2)
norm(node3-node1,2)-norm(node3f-node1f,2)
norm(node4-node1,2)-norm(node4f-node1f,2)
norm(node5-node1,2)-norm(node5f-node1f,2)
