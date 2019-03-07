% custom lagrangian formulation for the equations of motion

q = [0,0,0];

dq = [0,0,0];

M = [];
C = [];
g = [];

f = -b*dq; % any other friction models

% tau = 