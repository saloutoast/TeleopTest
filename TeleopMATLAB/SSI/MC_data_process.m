
%% open data from Logs folder
%test_data = importdata("../../3DOF_controller/Logs/150453_12_20_2018.log");

test1 = importdata("../../3DOF_controller/Logs/222414_01_14_2019.log");
test2 = importdata("../../3DOF_controller/Logs/223033_01_14_2019.log");

% for "222414...": mode, scaling*100, diffq[1]*1000, tau1[1]*1000, diffq[2]*1000, tau1[2]*1000
% for "223033...": mode, scaling*100, q1[1]*1000, diffq[1]*1000, tau1[1]*1000, diffdq[1]*1000, delO[1]*1000

%% split data to two cases
tPD1 = 0;
tSSI1 = 0;
for ii = 1:size(test1,1)
    if (test1(ii,1)==1)
        PD1(ii,:) = [tPD1, test1(ii,2)/100, test1(ii,3:end)/1000];
        tPD1 = tPD1 + 0.001;
    elseif (test1(ii,1)==3)
        SSI1((ii-size(PD1,1)),:) = [tSSI1, test1(ii,2)/100, test1(ii,3:end)/1000];
        tSSI1 = tSSI1 + 0.001;
    end        
end

tPD2 = 0;
tSSI2 = 0;
jj = 1;
for ii = 1:size(test2,1)
    if (test2(ii,1)==1)
        PD2(ii,:) = [tPD2, test2(ii,2)/100, test2(ii,3:end)/1000];
        tPD2 = tPD2 + 0.001;
    elseif (test2(ii,1)==3)
        SSI2(jj,:) = [tSSI2, test2(ii,2)/100, test2(ii,3:end)/1000, test2(ii+1,1)/1000];
        tSSI2 = tSSI2 + 0.002;
        jj = jj + 1;
    end        
end

%% stats and plots

% for PD1, SSI1: time, scaling, diffq[1], tau1[1], diffq[2], tau1[2]
% for PD2, SSI2: time, scaling, q1[1], diffq[1], tau1[1], diffdq[1], delO[1]

% figure;
% plot(PD1(:,1), PD1(:,3), PD1(:,1), PD1(:,5));
% figure;
% plot(SSI1(:,1), SSI1(:,3), SSI1(:,1),SSI1(:,5));

figure;
subplot(3,1,1);
plot(PD2(:,1), PD2(:,4), PD2(:,1), zeros(1,length(PD2(:,1)))); ylim([-1.5, 1.5]);
legend('delq','0');
title('Position');

subplot(3,1,2);
plot(PD2(:,1), PD2(:,6));
title('Velocity');

subplot(3,1,3);
plot(PD2(:,1), PD2(:,5).*(PD2(:,4)<0)); ylim([-0.02,0.02]);
title('Torque');


figure;
subplot(3,1,1);
plot(SSI2(:,1), SSI2(:,4), SSI2(:,1), zeros(1,length(SSI2(:,1)))); ylim([-0.02, 0.02]);
legend('delq','0');
title('Position');

subplot(3,1,2);
plot(SSI2(:,1), SSI2(:,6));
title('Velocity');

subplot(3,1,3);
plot(SSI2(:,1), SSI2(:,5), SSI2(:,1), SSI2(:,7)); ylim([-2, 2]);
legend('T','delO');
title('Torque');




% SSI_err = norm(SSI1(:,4)-SSI1(:,5))/(length(SSI1(:,4)));
% PD_err = norm(PD1(:,7))/(length(PD1(:,7)));

% figure; 
% subplot(2,1,1); xlim([-1.5, 2.5]); ylim([-1, 1]);
% subplot(2,1,2); xlim([0, 75]); ylim([-5, 5]);
% for jj=20011:10:size(SSI1,1)
%     subplot(2,1,1); hold on;
%     plot(SSI1(jj-10:jj,4)-SSI1(jj-10:jj,5), SSI1(jj-10:jj,9));
%     hold off;
%     subplot(2,1,2); hold on;
%     plot(SSI1(jj-10:jj,1),SSI1(jj-10:jj,4)-SSI1(jj-10:jj,5));
%     hold off;
%     drawnow
%     disp(jj);
% end

