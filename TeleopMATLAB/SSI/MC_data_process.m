
%% open data from Logs folder
test_data = importdata("../../3DOF_controller/Logs/150453_12_20_2018.log");

%% split data to two cases
tPD = 0;
tSSI = 0;
for ii = 1:size(test_data,1)
    if (test_data(ii,1)==1)
        PD_data(ii,:) = [tPD, test_data(ii,2)/100, test_data(ii,3:end)/1000];
        tPD = tPD + 0.001;
    else 
        SSI_data((ii-size(PD_data,1)),:) = [tSSI, test_data((ii-size(PD_data,1)),1)/100, test_data((ii-size(PD_data,1)),2:end)/1000];
        tSSI = tSSI + 0.001;
    end        
end

%% stats

SSI_err = mean(SSI_data(:,4)-SSI_data(:,5));
PD_err = mean(PD_data(:,7));

%% plots

figure; 
subplot(2,1,1); xlim([-1.5, 2.5]); ylim([-1, 1]);
subplot(2,1,2); xlim([0, 75]); ylim([-5, 5]);
for jj=20011:10:size(SSI_data,1)
    subplot(2,1,1); hold on;
    plot(SSI_data(jj-10:jj,4)-SSI_data(jj-10:jj,5), SSI_data(jj-10:jj,9));
    hold off;
    subplot(2,1,2); hold on;
    plot(SSI_data(jj-10:jj,1),SSI_data(jj-10:jj,4)-SSI_data(jj-10:jj,5));
    hold off;
    drawnow
    disp(jj);
end

