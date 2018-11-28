% SSI data processing

%% load data
SSI_data = load("SSI_test2.txt"); % Kv = 1 A/rad, alpha = 1*Kv, beta = 0.5*Kv
noSSI_data = load("noSSI_test2.txt"); % Ke = 1000 A/rad

%% scale data
SSI_data(:,1) = SSI_data(:,1)/1000; 
noSSI_data(:,1) = noSSI_data(:,1)/1000;

Imax = 1.5; % A
kt = 19.4; % in mNm/A
SSI_data(:,2) = ((SSI_data(:,2)-5000)/4000)*Imax*kt; % commanded T in mNm
noSSI_data(:,2) = ((noSSI_data(:,2)-5000)/4000)*Imax*kt;
SSI_data(:,4:5) = SSI_data(:,4:5)*(kt/1000); % SSI and VE T in mNm
noSSI_data(:,4:5) = noSSI_data(:,4:5)*(kt/1000);

%% plots for SSI

figure; hold on;% force for entire trial
plot(SSI_data(:,2)); % acutal force (limited by actuator)
plot(SSI_data(:,4)); % desired force
hold off;

figure; % position for entire trial
plot(SSI_data(:,1));

force_sec = SSI_data(6000:7200,2);
disp_sec = SSI_data(6000:7200,1);

figure; % section of force
plot(force_sec);

figure; % disp vs force
plot(disp_sec,force_sec);

figure; % rendered stiffness vs time
plot(force_sec./disp_sec);

%% plots for no_SSI

figure; % force for entire trial
plot(noSSI_data(:,2));

figure; % position for entire trial
plot(noSSI_data(:,1));

% force_sec2 = noSSI_data(3700:4100,2);
% disp_sec2 = noSSI_data(3700:4100,1);
% 
% figure; % section of force
% plot(force_sec2);
% 
% figure; % disp vs force
% plot(disp_sec2,force_sec2);
% 
% figure; % rendered stiffness vs time
% plot(force_sec2./disp_sec2);



