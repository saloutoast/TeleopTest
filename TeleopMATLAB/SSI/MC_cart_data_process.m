% script for processing and plotting 2DOF data from the MC arms

%% import and organize data

clear all
test = importdata("../../3DOF_controller/Logs/163716_01_30_2019.log");

% only want data from when scaling column is at desired value
% data is: time, q1[0], q1[1], q1[2], p1[0], p1[1], p1[2]
tPD = 0;
jj = 1;

for ii = 1:size(test,1)
    PD(jj,:) = [tPD, test(ii,2:end)/1000];
    tPD = tPD + 0.001;
    jj = jj + 1;       
end

%% add cartesian coords of end effector

% fix knee offset problem???
%PD(:,4) = -PD(:,4)+(2*-2.766);

Labad = 0.0577;
Lhip = 0.2088;
Lknee = 0.175;

s1 = sin(PD(:,2));
s2 = sin(PD(:,3));
s3 = sin(PD(:,4));

c1 = cos(PD(:,2));
c2 = cos(PD(:,3));
c3 = cos(PD(:,4));
     
c23 = c2.*c3 - s2.*s3;
s23 = s2.*c3 + c2.*s3;
     
% End effector position
PD_hip = [zeros(length(s1),1), Labad*c1, Labad*s1];
PD_knee = [Lhip*s2, Labad*c1 + Lhip*c2.*s1, Labad*s1 - Lhip*c1.*c2];
PD_end = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];

%% animation for both with circle motion

% vid = VideoWriter('MC_PDcycle.avi');
% open(vid)
fig1 = figure;
for ii=1800:25:size(PD_end,1)
    
    plot3([0, PD_hip(ii,2), PD_knee(ii,2), PD_end(ii,2)],[0, PD_hip(ii,3), PD_knee(ii,3), PD_end(ii,3)],[0, PD_hip(ii,1), PD_knee(ii,1), PD_end(ii,1)]);
    hold on
    plot3([0,PD_hip(ii,2),PD_knee(ii,2)],[0,PD_hip(ii,3),PD_knee(ii,3)],[0,PD_hip(ii,1),PD_knee(ii,1)],'*g','LineWidth',2);
    plot3(PD_end(ii,2),PD_end(ii,3),PD_end(ii,1),'*r','LineWidth',2);
    quiver3(PD_end(ii,2),PD_end(ii,3),PD_end(ii,1),PD(ii,6)/(10*norm(PD(ii,5:7))),PD(ii,7)/(10*norm(PD(ii,5:7))),PD(ii,5)/(10*norm(PD(ii,5:7))),'k','LineWidth',1.5);
    quiver3(PD_end(ii,2),PD_end(ii,3),PD_end(ii,1),PD(ii,6)/(10*norm(PD(ii,5:7))),0,0,'r','LineWidth',1.5);
    quiver3(PD_end(ii,2),PD_end(ii,3),PD_end(ii,1),0,PD(ii,7)/(10*norm(PD(ii,5:7))),0,'r','LineWidth',1.5);
    quiver3(PD_end(ii,2),PD_end(ii,3),PD_end(ii,1),0,0,PD(ii,5)/(10*norm(PD(ii,5:7))),'r','LineWidth',1.5);
   
    %     plot3(PD(ii,6),PD(ii,7),PD(ii,5),'*r','LineWidth',2);
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
    disp(ii)
%     F = getframe(fig1);
%     writeVideo(vid,F.cdata(:,:,:));
end
% close(vid)

%% plots

figure; subplot(1,3,1); 
plot(PD(:,1),PD(:,2));
title('q1[0]');
subplot(2,3,2);
plot(PD(:,1),PD(:,3));
title('q1[1]');
subplot(2,3,3);
plot(PD(:,1),PD(:,4));
title('q1[2]');
subplot(2,3,4);


