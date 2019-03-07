function [qnew, qdnew, qdd] = euler_dyn(robot, q, qd, tau, time)
% Called to calculate forward dynamics for one step of euler integration
%
% takes SerialLink object, current joint positions and velocities, current torque,
% and time (now as a step length, maybe as a vector later)
%
% returns new joint positions and velocities, assuming constant tau over
% the timestep
    
%     for ii = 1:10
    qdd = robot.accel(q, qd, tau);
    qd = qd + time*qdd';
    q = q + time*qd;
%     end
    
    qnew = q;
    qdnew = qd;
    qdd = qdd';
    
        
end
