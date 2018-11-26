% custom simulation solver script

% setup
kt = 19.4e-3; % torque constant in Nm/A
ke = (491)*((2*pi)/(360*60)); % speed constant in rad/s/V 
J = (68.1+80.65+(13.37*(2.2^2)))/(1000*100*100); % total inertia in kg*m2
b = J/10; % motor damping (guess)
KpC = 1; % value from ESCON current controller loop
R = 0.314; % terminal resistance in Ohms
L = 0.085e-3; % terminal inductance in H

Kp = 100; % unstable at Kp = 1000 if SSI = 0; but stable if SSI = 1
Kd = 35;
SSI = 1; % 1 if SSI is implemented
imax = 1; % maximum current
Vmax = 30; % maximum voltage

step = 0.001;
Tmax = 2;
time = 0:step:Tmax;

theta = zeros(4,length(time));
controls = zeros(3,length(time));
currents = zeros(2,length(time));
SSI_vals = zeros(1,length(time));
delO = 0;
ext_torques = zeros(2,length(time));
theta(:,1) = [0;0;2;0]; % theta0, thetadot0, theta1, thetadot1

for tt=2:length(time) % for each time step
    
    % calculate control currents, saturated at idmax
    id_temp = (-Kp*((theta(1,tt-1)-theta(3,tt-1))*(180/pi))-(Kd*(theta(2,tt-1)-theta(4,tt-1))))*(imax/4000);
%     if (abs(((theta(1,tt-1)-theta(3,tt-1))*(180/pi))) < 1) % 1 degree deadzone for control current
%         id = 0;
%     end
    
    % SSI code (1 is master, 0 is slave)
    xmax = (theta(3,tt-1)-theta(1,tt-1))*(180/pi);
    fmax = controls(2,tt-1)*kt;

    Eout = step*((Kp*xmax)+SSI_vals(tt-1))*(theta(4,tt-1)-theta(2,tt-1));
    if ( xmax > 0) % master ahead of slave
        delO = -(2/xmax)*(Eout - (xmax*fmax*0.5)); % positive
    else % slave ahead of master
        delO = (2/xmax)*(Eout - (xmax*fmax*0.5)); % negative
    end

    SSI_vals(tt) = delO;
    
%     delO = 0;
    id = max(min( (id_temp+(SSI*delO)) ,imax),-imax);
    controls(:,tt) = [id_temp; id; id_temp+(SSI*delO)];
    
    % calculate reference voltages for current controllers
%     Vrefs = max(min((KpC*(controls(:,tt)-currents(:,tt-1))),Vmax),-Vmax);
%     Vrefs = KpC*(controls(:,tt)-currents(:,tt-1));

    % calculate external torques
    Text0 = 0;
    Text1 = 0;
    % wall -> increase motor inertia and damping while in contact
    
    ext_torques(:,tt) = [Text0; Text1];
    
    % run one solver step
    % motor currents
%     idot = (1/L)*(Vrefs - ke*[theta(2,tt-1);theta(4,tt-1)] - R*currents(:,tt-1)); % so large can assume id = i
%     currents(:,tt) = max(min((currents(:,tt-1) + step*idot),imax),-imax);
%     current_derivs(:,tt-1) = idot;
%     currents(:,tt) = controls(:,tt);
        
    % motor positions
    theta_dot = [theta(2,tt-1);
                 (kt/J)*(controls(2,tt-1)) - (b/J)*theta(2,tt-1) + Text0;
                 theta(4,tt-1);
%                  (kt/J)*(currents(2,tt-1)) - (b/J)*theta(4,tt-1) + Text1]; % default
%                  4*cos(2*time(tt))]; % track reference
                 0]; % held constant (step)
    theta(:,tt) = theta(:,tt-1) + step*theta_dot;
    
end

figure;
subplot(2,2,1);
plot(time,theta(1,:),time,theta(3,:));
subplot(2,2,2);
plot(time,theta(2,:),time,theta(4,:));
subplot(2,2,3);
plot(time,controls(1,:),time,controls(2,:),time,controls(3,:));
subplot(2,2,4);
plot(time,SSI_vals);