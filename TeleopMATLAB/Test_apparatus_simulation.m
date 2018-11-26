%% parameter definitions for "test_apparatus.slx"

% control frequency
Tcontrol = 0.001; %1kHz

% in control loop
XScale = 180/pi; % scale from radians to degrees
VScale = 180/pi; % scale by sample time

Kp = 100; % proportional gain / coupling stiffness
Kd = 35; % derivative gain / coupling damping

% in motor blocks (from maxon datasheet for motor 273752)
KpC = 375; % value from ESCON current controller loop

R = 0.314; % terminal resistance in Ohms
L = 0.085e-3; % terminal inductance in H
b = 0; % damping in motor
J = (68.1+80.65+(13.37*(2.2^2)))/(1000*100*100); % total inertia in kg*m2

Kt = 19.4e-3; % torque constant in Nm/A
Ke = (491)*((2*pi)/(360*60)); % speed constant in rad/s/V 

%% run simulation

data_out = struct('Kp',0,'Kd',0,'stdev',[],'cur_data',struct('I0d',[],'I0',[],'I1d',[],'I1',[]),'pos_data',struct('Kp',0,'Kd',0,'stdev',0,'pos0data',[],'pos1data',[],'simtime',[]));
runs = 1;
num_Kp = 1;
num_Kd = 1;

disp('starting...');
for pp = 0:1000:2000
    data_out.Kp(num_Kp) = pp;
    num_Kp = num_Kp + 1;
    for dd = 0:100:200
        fprintf('Run %d\n', runs);
        Kp = pp;
        Kd = dd;
        warning('off','all');
        sim('Test_apparatus.slx');
        warning('on','all');
        err = pos1sim.data-pos0sim.data;
                
        if (pp==0)
            data_out.Kd(num_Kd) = dd;
            num_Kd = num_Kd + 1;
        end
        data_out.stdev(runs) = std(err); % standard deviation of the error
        data_out.pos_data(runs).pos0data = pos0sim.data;
        data_out.pos_data(runs).pos1data = pos1sim.data;
        data_out.pos_data(runs).simtime = pos0sim.time;
        data_out.pos_data(runs).Kp = pp;
        data_out.pos_data(runs).Kd = dd;
        data_out.pos_data(runs).stdev = std(err);
        
        data_out.cur_data(runs).I0d = currents_sim.data(:,1);
        data_out.cur_data(runs).I0 = currents_sim.data(:,2);
        data_out.cur_data(runs).I1d = currents_sim.data(:,3);
        data_out.cur_data(runs).I1 = currents_sim.data(:,4);
        
        runs = runs+1;  
    end
end
disp('...done!');
       
%% visualize "stability" in 3D

x = data_out.Kp;
y = data_out.Kd;
z1 = reshape(data_out.stability, [length(x),length(y)]);
mesh(x,y,z1);
xlabel('Kp');
ylabel('Kd');
zlabel('Standard dev between positions');


        
        
        
        
        
        
        
        
        