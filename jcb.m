clc
clear all

syms x y z bx by bz

pr = [x;y;z];
pb = [bx;by;bz];

h = norm(pr - pb); % for single beacon
J = jacobian(h,[x,y,z]);


beacons


% op_x = [0,0];
% subs(J, [x1,x2], op_x)
