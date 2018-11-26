%% motor simulation script

% setup
kt = 19.4e-3; % torque constant in Nm/A
Jmotor = (68.1+80.65+(13.37*(2.2^2)))/(1000*100*100); % total inertia in kg*m2
bmotor = Jmotor/10; % motor damping (guess)
Kp = 100;
Kd = 35;
idmax = 100; % maximum current

% initial conditions of system
tspan = 0:0.001:10;
theta_init = [-1;0;-1;0]; % theta0, vel0, theta1, vel1

% simulate system
[t,theta] = ode45(@(t,theta)thetafcn(t,theta,kt,Jmotor,bmotor,Kp,Kd,idmax),tspan,theta_init);

Kd = 0;
[t2,theta2] = ode45(@(t,theta)thetafcn(t,theta,kt,Jmotor,bmotor,Kp,Kd,idmax),tspan,theta_init);

figure;
subplot(1,2,1);
plot(t,theta(:,1),t,theta(:,3));
subplot(1,2,2);
plot(t,theta(:,2),t,theta(:,4));

figure;
subplot(1,2,1);
plot(t2,theta2(:,1),t2,theta2(:,3));
subplot(1,2,2);
plot(t2,theta2(:,2),t2,theta2(:,4));

% std(theta(:,1)-theta(:,3))
% std(theta2(:,1)-theta2(:,3))

% mean(abs(theta(:,1)-theta(:,3)))
% mean(abs(theta2(:,1)-theta2(:,3)))

% motor ode function
function thetadot = thetafcn(t,theta,kt,Jmotor,bmotor,Kp,Kd,idmax)

    % calculate control currents, saturated at idmax
    id = max(min(-Kp*((theta(1)-theta(3))*(180/pi))-Kd*(theta(2)-theta(4)),idmax),-idmax);
    
    Text0 = 0; %-(kt/Jmotor)*id;
    Text1 = 0;
        
    % put into differential equation
    thetadot(1) = theta(2);
    thetadot(2) = (kt/Jmotor)*id - (bmotor/Jmotor)*theta(2) + Text0;
    
    % SSI implementation
    
    
%     % wall?
%     if ((theta(1) > 0.7) && (theta(3) > 0.7))
%         Text0 = -(kt/Jmotor)*id;
%         bmotor0 = 10;
%     else
%         Text0 = 0;
%         bmotor0 = bmotor;
%     end
%     thetadot(2) = (kt/Jmotor)*id - (bmotor0/Jmotor)*theta(2) + Text0;
    
    % track reference of theta(3) = -cos(2*t)
    thetadot(3) = theta(4); 
    thetadot(4) = 4*cos(2*t);    
    
    % step input on motor1
%     thetadot(3) = 0;
%     thetadot(4) = 0;
    
    % normal operation
%     thetadot(3) = theta(4);
%     thetadot(4) = (kt/Jmotor)*(-id) - (bmotor/Jmotor)*theta(4) + Text1;
    
    thetadot = thetadot';

end