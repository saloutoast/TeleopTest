%% testing script for MC arms

% go back to ode45, use interp1 on tau function? not exactly true, but
% avoids discrete steps?

% no damping/friction, see what torques are necessary to follow a smoother version of the path?

master = build_arm('Master');
slave = build_arm('Slave');
slave.base = transl([-0.3, 0, 0]);
fprintf('Arms built!\n')

%% test center of mass values

[A,B] = master.fkine([0 0 0]);
L = master.links;

for ii=1:3
    figure;

    master.plot([0 0 0])  
    hold on
    drawnow
  
    B(ii).plot('rgb','length',0.3);
    r = L(ii).r;
    C(ii) = B(ii)*SE3(transl(r));
    C(ii).plot('length',0.3);
    drawnow
    hold off
    pause
    
end

%% test dynamics calculations / inertia parameters

% load and scale recorded data
rawdata = importdata("../../3DOF_controller/Logs/114905_02_18_2019.log");
% only want data from when scaling column is at desired value
% data is q2[0:2], tau2[0:2]
t = 0;
jj = 1;
for ii = 1:size(rawdata,1)
    if (rawdata(ii,1)==50)
        data(jj,:) = [t, rawdata(ii,2:end)/1000]; % append time      
        t = t + 0.001;
        jj = jj + 1;
    end
end

%make sure torques from robot are in the same direction as joint axes in the model
data(:,5) = -1*data(:,5);

        
% trim beginning and end when arm is not in use
data = [data(1:(11581-1620),1), data(1620:11580,2:end)];



% arm animation with joint position data
qplot = data(1:50:end,2:4);
% figure;
% master.plot(qplot)

% calculate forward dynamics using only position data

% [q,qd,qdd] = jtraj(q0,qf,T,qd0,qdf) ... use 10 steps between each data point?
% then tau= R.rne(q,qd,qdd,options) ... make sure to specify gravity correctly


% compare to commanded torques

% plot tau from rne vs data(:,5:7)


%% simulate kinematics with commanded torques

master = build_arm('Master');
fprintf('Re-built arm\n');

tau_com = data(:,5:7);
Tmax = data(end,1);
step = 0.001;
Tsim = 3;
qinit = data(1,2:4);
joints = [1,0,0];

ti = 0:step:Tsim;
q_dyn = zeros(length(ti),3);
qd_dyn = zeros(length(ti),3);
qdd_dyn = zeros(length(ti),3);
% tau_test = ones(length(ti),3);

tau_mean = movmean(data(:,5:7),[6 6],1);

q_dyn(1,:) = qinit;
% initial vel is 0

disp('Simulating kinematics...');
% [ti,q_dyn,qd_dyn] = master.fdyn(1.0,@tau_func,qinit,[0,0,0],data,joints);
for ii=2:(length(ti)-1)
    
    if mod(ii,100)==0
        disp(ii)
    end
    [qnew, qdnew, qdd] = euler_dyn(master,q_dyn(ii-1,:),qd_dyn(ii-1,:),tau_mean(ii,:),step);
    q_dyn(ii,:) = qnew;
    qd_dyn(ii,:) = qdnew;
    qdd_dyn(ii-1,:) = qdd; % since qdd is from the start of the step

end

disp('Ready for plotting...')

% compare to measured positions
figure; plot(data(1:(Tsim*1000),1),data(1:(Tsim*1000),2),ti,q_dyn(:,1)); title('Ab-ad');
figure; plot(data(1:(Tsim*1000),1),data(1:(Tsim*1000),3),ti,q_dyn(:,2)); title('Hip');
figure; plot(data(1:(Tsim*1000),1),data(1:(Tsim*1000),4),ti,q_dyn(:,3)); title('Knee');

%% plotting

% t_deval = 0:0.001:Tsim;
% q_dyn_deval = interp1(ti,q_dyn,t_deval);

figure; 
master.plot(qinit);
% hold on;
% slave.plot(qinit);
for ii=25:25:(Tsim/step)
    master.plot(q_dyn(ii,:))
%     slave.plot(data(ii,2:4))
end
hold off;

%%


%% other

% figure;
% master.plot([0 3.493 -2.766])
% 
% master.teach()

% TODO: second script to calculate dynamics/trajectories
%
% - have master follow joint trajectory, slave move with PD controller
% - for SVM training, use 'ctraj' to generate trajectories between two
% points for master, simulate dynamics, have slave follow with PD
% controller while recording dq, dqdot, torques, etc.
% - play around with an averaging window for data fed into SVM
% - find a good way to test SVM performance in simulation