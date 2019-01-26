% script for processing and plotting 2DOF data from the MC arms

%% import and organize data

test = importdata("../../3DOF_controller/Logs/172442_01_25_2019.log");

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

%  const float s1 = sinf(q[0]);
%  const float s2 = sinf(q[1]);
%  const float s3 = sinf(q[2]);
%  const float c1 = cosf(q[0]);
%  const float c2 = cosf(q[1]);
%  const float c3 = cosf(q[2]);
% 
%  const float c23 = c2*c3 - s2*s3;
%  const float s23 = s2*c3 + c2*s3;
% 
%  /// End effector position
%  p[0] = L3*s23 + L2*s2;
%  p[1] = L1*c1 + L3*s1*c23 + L2*c2*s1;
%  p[2] = L1*s1 - L3*c1*c23 - L2*c1*c2;

% fix knee offset problem
PD(:,4) = -PD(:,4)+(2*-2.766);

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

% maybe a live animation of trajectory would be good, as well as error plots and statistics
% vid = VideoWriter('MC_PDcycle.avi');
% open(vid)
fig1 = figure;
for ii=1:100:size(PD_end,1)
    
    plot3([0, PD_hip(ii,2), PD_knee(ii,2), PD_end(ii,2)],[0, PD_hip(ii,3), PD_knee(ii,3), PD_end(ii,3)],[0, PD_hip(ii,1), PD_knee(ii,1), PD_end(ii,1)]);
    hold on
    plot3([0,PD_hip(ii,2),PD_knee(ii,2)],[0,PD_hip(ii,3),PD_knee(ii,3)],[0,PD_hip(ii,1),PD_knee(ii,1)],'*g','LineWidth',2);
    plot3(PD_end(ii,2),PD_end(ii,3),PD_end(ii,1),'*r','LineWidth',2);
    hold off
    axis equal
    grid on
    xlim([-0.5, 0.5]);
    ylim([-0.5, 0.5]);
    zlim([-0.5, 0.5]);
    title('PD');
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    
%     view([0, 90]);
    
    drawnow
%     F = getframe(fig1);
%     writeVideo(vid,F.cdata(:,:,:));
end
% close(vid);

% vid = VideoWriter('MC_SSIcycle.avi');
% open(vid);
% fig2 = figure;
% for ii=1:10:size(SSI_cart,1)
%     plot([0,SSI_cart(ii,1),SSI_cart(ii,5)],[0,SSI_cart(ii,3),SSI_cart(ii,7)], ...
%         [0,SSI_cart(ii,2),SSI_cart(ii,6)],[0,SSI_cart(ii,4),SSI_cart(ii,8)]);
%     axis equal
%     xlim([-0.4, 0.1]);
%     ylim([-0.1, 0.4]);
%     title('SSI');
%     xlabel('x (m)');
%     ylabel('y (m)');
%     drawnow
% %     F = getframe(fig2);
% %     writeVideo(vid,F.cdata(:,:,:));
% end
% close(vid);

%% plots

figure; subplot(1,3,1);
plot(PD(:,1),PD(:,5),PD(:,1),PD_end(:,1));
title('z?');
subplot(1,3,2);
plot(PD(:,1),PD(:,6),PD(:,1),PD_end(:,2));
title('x?');
subplot(1,3,3);
plot(PD(:,1),PD(:,7),PD(:,1),PD_end(:,3));
title('y?');