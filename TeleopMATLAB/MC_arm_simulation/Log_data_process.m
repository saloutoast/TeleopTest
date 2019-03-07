% script for processing and plotting 2DOF data from the MC arms

%% import and organize data

clear all
test = importdata("../../3DOF_controller/Logs/115759_02_28_2019.log");



% only want data from when scaling column is at desired value
% data is: time, q1[0], q1[1], q1[2], dq1[0], dq1[1], dq1[2]
tPD = 0;

jj = 1;

for ii = 1:size(test,1)
    data(ii,:) = [tPD, test(ii,2:end)/1000];
    tPD = tPD + 0.001;
end

%% add cartesian coords of end effector

% fix knee offset problem???
%PD(:,4) = -PD(:,4)+(2*-2.766);

Labad = 0.0577;
Lhip = 0.2088;
Lknee = 0.175;

s1 = sin(data(:,2));
s2 = sin(data(:,3));
s3 = sin(data(:,4));

c1 = cos(data(:,2));
c2 = cos(data(:,3));
c3 = cos(data(:,4));
     
c23 = c2.*c3 - s2.*s3;
s23 = s2.*c3 + c2.*s3;
     
% End effector position
pos_hip = [zeros(length(s1),1), Labad*c1, Labad*s1];
pos_knee = [Lhip*s2, Labad*c1 + Lhip*c2.*s1, Labad*s1 - Lhip*c1.*c2];
pos_end = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];


%% calculate endpoint forces from torques

J = [0, L3*c23 + L2*c2, L3*c23;
     L3*c1*c23 + L2*c1*c2 - L1*s1, -L3*s1*s23 - L2*s1*s2, -L3*s1*s23;
     L3*s1*c23 + L2*c2*s1 + L1*c1, L3*c1*s23 + L2*c1*s2, L3*c1*s23];

%% animation

% vid = VideoWriter('MC_PDcycle.avi');
% open(vid)
fig1 = figure;
fig2 = figure;
for ii=1:100:size(pos_end,1)
    
    figure(1);
    plot3([0, pos_hip(ii,2), pos_knee(ii,2), pos_end(ii,2)],[0, pos_hip(ii,3), pos_knee(ii,3), pos_end(ii,3)],[0, pos_hip(ii,1), pos_knee(ii,1), pos_end(ii,1)]);
    hold on
    plot3([0,pos_hip(ii,2),pos_knee(ii,2)],[0,pos_hip(ii,3),pos_knee(ii,3)],[0,pos_hip(ii,1),pos_knee(ii,1)],'*g','LineWidth',2);
    
    plot3(pos_end(ii,2),pos_end(ii,3),pos_end(ii,1),'*r','LineWidth',2);
    plot3(data(ii,6),data(ii,7),data(ii,5),'*g','LineWidth',2);
        
%     J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
%         Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
%         Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
% 
%     Ftip = pinv(J')*[data(ii,5);data(ii,6);data(ii,7)];
% 
%     quiver3(pos_end(ii,2),pos_end(ii,3),pos_end(ii,1),Ftip(2)/norm(Ftip),Ftip(3)/norm(Ftip),Ftip(1)/norm(Ftip),'r','LineWidth',1.5);
%     
%     
   

    hold off
    axis equal
    grid on
    xlim([-0.3, 0.3]);
    ylim([-0.3, 0.3]);
    zlim([-0.3, 0.3]);
    title('PD');
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    
%     view([90, 0]);
    
    drawnow
    
%     figure(2);
%     master.plot(data(ii,2:4))
    
    disp(ii)
%     F = getframe(fig1);
%     writeVideo(vid,F.cdata(:,:,:));
end
% close(vid)

%% velocity at tip
Vtip = [];

for ii=1:1:size(pos_end,1)
    J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
        Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
        Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
    
    Vtip(ii,:) = (pinv(J')*[data(ii,5);data(ii,6);data(ii,7)])';
end

%% plots

% figure; 
% yyaxis right
% plot(data(:,1),Ftip(:,1),'LineWidth',1.5);
% ylabel('Force in z-direction at end-effector');
% yyaxis left
% hold on
% plot(data(:,1),pos_end(:,1),'LineWidth',1.5);
% plot(data(:,1),-0.05*ones(32039,1),'-.k');
% hold off
% ylabel('Position along z-direction of end-effector');


% test outputs of collision detection here
mmback_hip = movmean(data(:,6), [5, 0]);
mmfor_hip = movmean(data(:,6), [0, 5]);
col_det = zeros(size(mmback_hip));

% use probabilities instead of absolute difference
% use endpoint velocity instead of joint velocities?

for ii = 2:length(col_det)
%     col_det(ii) = abs(mm(ii-1)-data(ii,6));
    if (abs(mmback_hip(ii)-mmfor_hip(ii)) > 0.35) && (abs(mmback_hip(ii)) > 0.1) && (abs(mmfor_hip(ii)) < 0.1)
        col_det(ii) = 1;
    end
end

figure; yyaxis left; hold on; plot(data(:,1),data(:,6)); plot(data(:,1),mmfor_hip, 'g'); plot(data(:,1),mmback_hip, 'k');
yyaxis right; hold on; plot(data(:,1), pos_end(:,1)); plot(data(:,1), col_det, 'r');
title('Collision detection on hip joint');

mmback_z = movmean(Vtip(:,1), [10, 0]);
mmfor_z = movmean(Vtip(:,1), [0, 5]);
col_detz = zeros(size(mmback_z));

% use probabilities instead of absolute difference
% use endpoint velocity instead of joint velocities?

for ii = 2:length(col_detz)
%     col_det(ii) = abs(mm(ii-1)-data(ii,6));
    if (abs(mmback_z(ii)-mmfor_z(ii)) > 1.0) && (abs(mmback_z(ii)) > 1.2) && (abs(mmfor_z(ii)) < 0.2)
        col_detz(ii) = 1;
    end
end

figure; yyaxis left; hold on; plot(data(:,1),Vtip(:,1)); plot(data(:,1),mmfor_z, 'g'); plot(data(:,1),mmback_z, 'k');
yyaxis right; hold on; plot(data(:,1), pos_end(:,1)); plot(data(:,1), col_detz, 'r');
title('Collision detection on z-velocity');






