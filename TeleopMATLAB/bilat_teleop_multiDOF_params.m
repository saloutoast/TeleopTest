% parameter definitions for "bilat_teleop_multiDOF.m"

% master state-space dynamics
Am = [1 0; 0 1];
Bm = [1 0; 0 1];

% slave state-space dynamics
As = [1 0; 0 1];
Bs = [1 0; 0 1];

% operator impedance values
Kh = [0 0; 0 0];
Bh = [0 0; 0 0];
Mh = [0 0; 0 0];

% environment impedance values
Ke = [0 0; 0 0];
Be = [0 0; 0 0];
Me = [0 0; 0 0];

