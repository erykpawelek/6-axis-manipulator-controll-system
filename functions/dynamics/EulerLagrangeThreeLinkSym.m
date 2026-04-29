function [] = EulerLagrangeThreeLinkSym()
% Define real symbolic variables for mass, inertia, geometry, and gravity
syms m1 m2 m3 J1xx J1yy J1zz J2xx J2yy J2zz J3xx J3yy J3zz p1 p2 p3 L2 g t real

% Define real symbolic variables for kinematic parameters (displacement, velocity, acceleration)
syms th1 th2 th3 dth1 dth2 dth3 ddth1 ddth2 ddth3 real

% Kinematic parameters vectors (initialized as 1x3 row vectors)
q = [th1, th2, th3];
dq = [dth1, dth2, dth3];
ddq = [ddth1, ddth2, ddth3];

% Change of names of kinematic parameters to time-dependent functions
qt = [str2sym('th1(t)'), str2sym('th2(t)'), str2sym('th3(t)')];
dqt = [str2sym('dth1(t)'), str2sym('dth2(t)'), str2sym('dth3(t)')];
dqtt = [str2sym('diff(th1(t),t)'), str2sym('diff(th2(t),t)'), str2sym('diff(th3(t),t)')];
ddqt = [str2sym('diff(dth1(t),t)'), str2sym('diff(dth2(t),t)'), str2sym('diff(dth3(t),t)')];

% Corrected kinetic energy equations for 3 first link of manipulator with
% assumption that we use Koenig's convention.
% Link 1: 
Ek1 = 1/2*J1zz*dth1^2;
% Link 2: 
Ek2 = 1/2*m2*((p2*dth2)^2 + (p2*sin(th2)*dth1)^2) + 1/2*J2yy*dth2^2 + 1/2*(J2xx*sin(th2)^2 + J2zz*cos(th2)^2)*dth1^2;
% Link 3: 
X3 = L2*sin(th2) + p3*sin(th2+th3);
X3d = L2*cos(th2)*dth2 + p3*cos(th2 + th3)*(dth2 + dth3);
Y3d = -L2*sin(th2)*dth2 - p3*sin(th2 + th3)*(dth2 + dth3);
Ek3 = 1/2*m3*(X3d^2 + Y3d^2 + (X3*dth1)^2) + 1/2*J3yy*(dth2+dth3)^2 + 1/2*(J3xx*sin(th2+th3)^2 + J3zz*cos(th2+th3)^2)*dth1^2;
% Total kinetic energy
Ek = simplify(Ek1 + Ek2 + Ek3);

% Potential energies:
% Link 1: 
Ep1 = 0;
% Link 2: 
Ep2 = m2*g*p2*cos(th2); % In reference pose manipulator 2nd link is pointing upwards
% Link 3: 
Ep3 = m3*g*(L2*cos(th2)+p3*cos(th2+th3));
% Total potential energy
Ep = simplify(Ep1 + Ep2 + Ep3);

% Lagrange function L
L = Ek - Ep;

% Derivative of L with respect to dq (velocity)
f1 = jacobian(L, dq);

% Introduction of time variable t
f2 = subs(f1, q, qt);
f3 = subs(f2, dq, dqt);

% Derivative of f3 with respect to t
f4 = diff(f3, t);

% Unification of denotations and removing of time variable t
f5 = subs(f4, ddqt, ddq);
f6 = subs(f5, dqt, dq);
f7 = subs(f6, dqtt, dq);
f8 = subs(f7, qt, q);

% Derivative of L with respect to q (displacement)
f9 = jacobian(L, q);

% Setting of the right-hand side of Dynamic Equations of Motion (DEM)
% DEM results in a 1x3 row vector
DEM = f8 - f9;

% Extracting Mass Matrix (M) by differentiating DEM with respect to ddq
% M results in a 3x3 matrix
M = jacobian(DEM, ddq);

% Calculate the combined Coriolis, Centrifugal, and Gravity vector (C+G)
% DEM and ddq are transposed to 3x1 column vectors to match M's dimensions
CG = simplify(DEM' - M * ddq');

% Isolate the Gravity vector (G) by substituting 0 for all velocities in the dq vector
G = simplify(subs(CG, dq, [0, 0, 0]));

% Isolate the Coriolis and Centrifugal vector (C) by subtracting G from the combined CG vector
C = simplify(CG - G);

% Define symbolic variables for the left-hand side input generalized forces
syms tau1 tau2 tau3 real
Tau = [tau1; tau2; tau3];

% Transpose the acceleration row vector to a column vector for matrix multiplication display
ddq_col = ddq';

% Display the final results in the console matching the requested matrix equation format
disp('--- Dynamic Equations of Motion [ Tau = M*ddq + C + G ] ---');
disp('Left-Hand Side Vector (Tau):');
disp(Tau);
disp('Mass Matrix (M):');
disp(M);
disp('Acceleration Vector (ddq):');
disp(ddq_col);
disp('Coriolis and Centrifugal Vector (C):');
disp(C);
disp('Gravity Vector (G):');
disp(G);
end