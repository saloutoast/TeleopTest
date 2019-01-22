% script for processing and plotting 2DOF data from the MC arms

%% import and organize data

test = importdata("../../3DOF_controller/Logs/141607_01_22_2019.log");

% only want data from when scaling column is at desired value
% data is: time, q1[1], q2[1], tau2[1], q1[2], q2[2], tau2[2]
tPD = 0;
tSSI = 0;
jj = 1;
kk = 1;

for ii = 1:size(test,1)
    if ((test(ii,1)==1)) %&&(test(ii,2)==50))
        PD(jj,:) = [tPD, test(ii,3:end)/1000];
        tPD = tPD + 0.001;
        jj = jj + 1;
    elseif ((test(ii,1)==3))%&&(test(ii,2)==50))
        SSI(kk,:) = [tSSI, test(ii,3:end)/1000];
        tSSI = tSSI + 0.001;
        kk = kk + 1;
    end        
end

%% add cartesian coords of end effector

Lhip = 0.2088;
Lknee = 0.175;

s1 = [sin(PD(:,2)), sin(PD(:,3))]; % [sin(q1[1]), sin(q2[1])]
s2 = [sin(-PD(:,5)+(pi/6)), sin(-PD(:,6)+(pi/6))]; % [sin(q1[2]), sin(q2[2])]

c1 = [cos(PD(:,2)), cos(PD(:,3))];
c2 = [cos(-PD(:,5)+(pi/6)), cos(-PD(:,6)+(pi/6))];
     
c12 = c1.*c2 - s1.*s2;
s12 = s1.*c2 + c1.*s2;
     
% End effector position
PD_cart = [Lhip*c1, Lhip*s1, (Lhip*c1 + Lknee*c12), (Lhip*s1 + Lknee*s12)]; % [xk1, xk2, yk1, yk2, xe1, xe2, ye1, ye2]
 
s1 = [sin(SSI(:,2)), sin(SSI(:,3))]; % [sin(q1[1]), sin(q2[1])]
s2 = [sin(-SSI(:,5)+(pi/6)), sin(-SSI(:,6)+(pi/6))]; % [sin(q1[2]), sin(q2[2])]

c1 = [cos(SSI(:,2)), cos(SSI(:,3))];
c2 = [cos(-SSI(:,5)+(pi/6)), cos(-SSI(:,6)+(pi/6))];
     
c12 = c1.*c2 - s1.*s2;
s12 = s1.*c2 + c1.*s2;
     
% End effector position
SSI_cart = [Lhip*c1, Lhip*s1, (Lhip*c1 + Lknee*c12), (Lhip*s1 + Lknee*s12)]; % [xk1, xk2, yk1, yk2, xe1, xe2, ye1, ye2]


%% animation for both with circle motion

% maybe a live animation of trajectory would be good, as well as error plots and statistics
% vid = VideoWriter('MC_PDcycle.avi');
% open(vid)
% fig1 = figure;
% for ii=1:10:size(PD_cart,1)
%     plot([0,PD_cart(ii,1),PD_cart(ii,5)],[0,PD_cart(ii,3),PD_cart(ii,7)], ...
%         [0,PD_cart(ii,2),PD_cart(ii,6)],[0,PD_cart(ii,4),PD_cart(ii,8)]);
%     axis equal
%     xlim([-0.4, 0.1]);
%     ylim([-0.1, 0.4]);
%     title('PD');
%     xlabel('x (m)');
%     ylabel('y (m)');
%     drawnow
% %     F = getframe(fig1);
% %     writeVideo(vid,F.cdata(:,:,:));
% end
% close(vid);

% vid = VideoWriter('MC_SSIcycle.avi');
% open(vid);
fig2 = figure;
for ii=16000:10:size(SSI_cart,1)
    plot([0,SSI_cart(ii,1),SSI_cart(ii,5)],[0,SSI_cart(ii,3),SSI_cart(ii,7)], ...
        [0,SSI_cart(ii,2),SSI_cart(ii,6)],[0,SSI_cart(ii,4),SSI_cart(ii,8)]);
    axis equal
    xlim([-0.4, 0.1]);
    ylim([-0.1, 0.4]);
    title('SSI');
    xlabel('x (m)');
    ylabel('y (m)');
    drawnow
%     F = getframe(fig2);
%     writeVideo(vid,F.cdata(:,:,:));
end
% close(vid);

%% animation for SSI with jerky motion

% vid = VideoWriter('MC_SSIjerk.avi');
% open(vid);
% fig = figure;
% for ii= 10000:10:size(SSI_cart,1)
%     plot([0,SSI_cart(ii,1),SSI_cart(ii,5)],[0,SSI_cart(ii,3),SSI_cart(ii,7)], ...
%         [0,SSI_cart(ii,2),SSI_cart(ii,6)],[0,SSI_cart(ii,4),SSI_cart(ii,8)]);
%     axis equal
%     xlim([-0.4, 0.1]);
%     ylim([-0.1, 0.4]);
%     %legend('M','S');
%     title('SSI');
%     xlabel('x (m)');
%     ylabel('y (m)');
%     drawnow
%     F = getframe(fig);
%     writeVideo(vid,F.cdata(:,:,:));
% end
% close(vid)

%% static plots

figure;
plot(PD(:,1),PD(:,2)-PD(:,3),PD(:,1),PD(:,5)-PD(:,6));
legend('Hip','Knee');
xlabel('Time (s)'); ylabel('Position (rad)');
title('PD position');

figure;
plot(SSI(:,1), SSI(:,2)-SSI(:,3), SSI(:,1), SSI(:,5)-SSI(:,6));
legend('Hip','Knee');
xlabel('Time (s)'); ylabel('Position (rad)');
title('SSI position');

figure;
plot(PD(:,1),PD(:,4),PD(:,1),PD(:,7));
legend('Hip','Knee');
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('PD slave force');

figure;
plot(SSI(:,1),SSI(:,4),SSI(:,1),SSI(:,7));
legend('Hip','Knee');
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('SSI slave force');

%% stats

% average error:
PDerr = [mean(abs(PD(1:9000,2)-PD(1:9000,3))), mean(abs(PD(1:9000,5)-PD(1:9000,6)))];

SSIerr = [mean(abs(SSI(1:10000,2)-SSI(1:10000,3))), mean(abs(SSI(1:10000,5)-SSI(1:10000,6)))];

SSIerr2 = [mean(abs(SSI(10000:end,2)-SSI(10000:end,3))), mean(abs(SSI(10000:end,5)-SSI(10000:end,6)))];

disp(PDerr); disp(SSIerr); disp(SSIerr2);

