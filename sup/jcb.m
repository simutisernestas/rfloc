clc
clear

syms x y z bx by bz

pr = [x;y;z];
pb = [bx;by;bz];

h = sqrt(sum((pr-pb).^2)) %norm(pr-pb,2) % for single beacon
dh = simplify(diff(h))

J = jacobian(h,[x,y,z]);

% beacons

% op_x = [0,0];
% subs(J, [x1,x2], op_x)
