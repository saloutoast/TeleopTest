% parameter definitions for "sampled_data_bilat_teleop_time_delay.slx"

% sample time
tsamp = 0.005;

% delay times;
T1 = 0.001; % master to slave
T2 = 0.001; % slave to master

% operator impedance
Mh = 0.1;
Bh = 0.1;
Kh = 0;
xvh = 0;

% environment impedance
Me = 0;
Be = 0;
Ke = 0;
xve = 0;

% master dynamics
Mm = 0.015;
Bm = 0.1;

% slave dynamics
Ms = 0.015;
Bs = 0.1;

% PID control gains
Kp = 5;
Ki = 0;
Kd = 0;

% simulate
sim("sampled_data_bilat_teleop_time_delay.slx");
