% script for processing and plotting 2DOF data from the MC arms

%% import and organize data

clear all

test = importdata("../../3DOF_controller/Logs/151704_01_29_2019.log");
% test2 = importdata("../../3DOF_controller/Logs/141313_01_29_2019.log");

% only want data from when scaling column is at desired value
% data is: time, q1[1], q2[1], tau2[1], q1[2], q2[2], tau2[2]
tPD = 0;
tSSI = 0;
jj = 1;
kk = 1;

for ii = 1:size(test,1)
    if ((test(ii,1)==1)) %&&(test(ii,2)==50))
        PD(jj,:) = [tPD, test(ii,3:end)/1000, test(ii,2)/100];
        tPD = tPD + 0.001;
        jj = jj + 1;
    elseif ((test(ii,1)==3))%&&(test(ii,2)==50))
        SSI(kk,:) = [tSSI, test(ii,3:end)/1000, test(ii,2)/100];
        tSSI = tSSI + 0.001;
        kk = kk + 1;
    end        
end

% tPD = 0;
% tSSI = 0;
% jj = 1;
% kk = 1;
% 
% for ii = 1:size(test2,1)
%     if ((test2(ii,1)==1)) %&&(test(ii,2)==50))
%         PD2(jj,:) = [tPD, test2(ii,3:end)/1000, test2(ii,2)/100];
%         tPD = tPD + 0.001;
%         jj = jj + 1;
%     elseif ((test2(ii,1)==3))%&&(test(ii,2)==50))
%         SSI2(kk,:) = [tSSI, test2(ii,3:end)/1000, test2(ii,2)/100];
%         tSSI = tSSI + 0.001;
%         kk = kk + 1;
%     end        
% end

%% add cartesian coords of end effector

Lhip = 0.2088;
Lknee = 0.175;

PD(:,5) = -PD(:,5)+(2*-2.766);
PD(:,6) = -PD(:,6)+(2*-2.766);

s1 = [sin(PD(:,2)), sin(PD(:,3))]; % [sin(q1[1]), sin(q2[1])]
s2 = [sin(PD(:,5)), sin(PD(:,6))]; % [sin(q1[2]), sin(q2[2])]

c1 = [cos(PD(:,2)), cos(PD(:,3))];
c2 = [cos(PD(:,5)), cos(PD(:,6))];
     
c12 = c1.*c2 - s1.*s2;
s12 = s1.*c2 + c1.*s2;
     
% End effector position
PD_cart = [Lhip*c1, Lhip*s1, (Lhip*c1 + Lknee*c12), (Lhip*s1 + Lknee*s12)]; % [xk1, xk2, yk1, yk2, xe1, xe2, ye1, ye2]
 
SSI(:,5) = -SSI(:,5)+(2*-2.766);
SSI(:,6) = -SSI(:,6)+(2*-2.766);

s1 = [sin(SSI(:,2)), sin(SSI(:,3))]; % [sin(q1[1]), sin(q2[1])]
s2 = [sin(SSI(:,5)), sin(SSI(:,6))]; % [sin(q1[2]), sin(q2[2])]

c1 = [cos(SSI(:,2)), cos(SSI(:,3))];
c2 = [cos(SSI(:,5)), cos(SSI(:,6))];
     
c12 = c1.*c2 - s1.*s2;
s12 = s1.*c2 + c1.*s2;
     
% End effector position
SSI_cart = [Lhip*c1, Lhip*s1, (Lhip*c1 + Lknee*c12), (Lhip*s1 + Lknee*s12)]; % [xk1, xk2, yk1, yk2, xe1, xe2, ye1, ye2]


%% animation for both with circle motion

% maybe a live animation of trajectory would be good, as well as error plots and statistics
% vid = VideoWriter('MC_PDcycle.avi');
% open(vid)
fig1 = figure;
for ii=1:10:size(PD_cart,1)
    plot([0,PD_cart(ii,1),PD_cart(ii,5)],[0,PD_cart(ii,3),PD_cart(ii,7)], ...
        [0,PD_cart(ii,2),PD_cart(ii,6)],[0,PD_cart(ii,4),PD_cart(ii,8)]);
    axis equal
    xlim([-0.4, 0.1]);
    ylim([-0.1, 0.4]);
    title('PD');
    xlabel('x (m)');
    ylabel('y (m)');
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

%% static plots

% figure;
% subplot(3,1,1);
% plot(PD(:,1),PD(:,2)-PD(:,3),PD(:,1),PD(:,5)-PD(:,6));
% legend('Hip','Knee');
% xlabel('Time (s)'); ylabel('Position (rad)');ylim([-1, 1]);
% title('PD position');
% subplot(3,1,2);
% plot(PD(:,1),PD(:,4),PD(:,1),PD(:,7));
% legend('Hip','Knee');
% xlabel('Time (s)'); ylabel('Torque (Nm)');
% title('PD slave force');
% subplot(3,1,3);
% plot(PD(:,1),PD(:,8));
% title('Gain Scaling');
% xlabel('Time (s)');

figure;
subplot(4,1,1)
plot(SSI(:,1), SSI(:,2), SSI(:,1), SSI(:,5));
legend('Hip','Knee');
xlabel('Time (s)'); ylabel('Position (rad)'); ylim([-.1, .1]);
title('SSI position');
subplot(4,1,2);
plot(SSI(:,1),SSI(:,3),SSI(:,1),SSI(:,6));
legend('Hip','Knee');
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('SSI delO');
subplot(4,1,3);
plot(SSI(:,1),SSI(:,4),SSI(:,1),SSI(:,4)-SSI(:,3),SSI(:,1),SSI(:,7),SSI(:,1),SSI(:,7)+SSI(:,6));
legend('Hip','Hip no delO','Knee','Knee no delO');
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('SSI slave force');
subplot(4,1,4);
plot(SSI(:,1),SSI(:,8));
title('Gain Scaling');
xlabel('Time (s)');

% figure;
% subplot(3,1,1)
% plot(SSI2(:,1), SSI2(:,2)-SSI2(:,3), SSI2(:,1), SSI2(:,5)-SSI2(:,6));
% legend('Hip','Knee');
% xlabel('Time (s)'); ylabel('Position (rad)'); ylim([-.1, .1]);
% title('SSI2 position');
% subplot(3,1,2);
% plot(SSI2(:,1),SSI2(:,4),SSI2(:,1),SSI2(:,7));
% legend('Hip','Knee');
% xlabel('Time (s)'); ylabel('Torque (Nm)');
% title('SSI2 slave force');
% subplot(3,1,3);
% plot(SSI2(:,1),SSI2(:,8));
% title('Gain Scaling');
% xlabel('Time (s)');



%% for velocities...

PDvel = [0,0,0,0];
for ii=2:size(PD2,1)
    PDvel(ii,:) = (1/0.001)*[PD2(ii,2)-PD2(ii-1,2), PD2(ii,3)-PD2(ii-1,3), PD2(ii,5)-PD2(ii-1,5), PD2(ii,6)-PD2(ii-1,6)];
end

delv2 = PDvel(:,1)-PDvel(:,2);

std(delv2(1:12000))
std(delv2(12000:22000))

% figure;
% subplot(3,1,1);
% plot(PD2(:,1),PDvel(:,1),PD2(:,1),PDvel(:,2));
% subplot(3,1,2);
% plot(PD2(:,1),delv2);
% subplot(3,1,3);
% plot(PD2(:,1),PD2(:,8));

delv = PD(:,2)-PD(:,3);

std(delv(1:7500))
std(delv(10000:26000))

filt_vel = [PD(1,2),PD(1,3),PD(1,5),PD(1,6)];
for ii=2:length(delv)
    filt_vel(ii,:) = 0.5*filt_vel(ii-1,:) + 0.5*([PD(ii,2),PD(ii,3),PD(ii,5),PD(ii,6)]);
    
%     if ii<=10
%         filt_vel(ii,:) = [PD(ii,2),PD(ii,3),PD(ii,5),PD(ii,6)];
%     else
%         filt_vel(ii,:) = 0.1 * (sum(filt_vel(ii-9:ii-1,:),1) + [PD(ii,2),PD(ii,3),PD(ii,5),PD(ii,6)]);
%     end
    
end

figure;
subplot(3,1,1);
plot(PD(:,1),PD(:,2),PD(:,1),PD(:,3));
subplot(3,1,2);
plot(PD(:,1),delv,PD(:,1),filt_vel(:,1)-filt_vel(:,2));
subplot(3,1,3);
plot(PD(:,1),PD(:,8));

filt_delv = filt_vel(:,1)-filt_vel(:,2);

std(filt_delv(1:7500))
std(filt_delv(10000:26000))
