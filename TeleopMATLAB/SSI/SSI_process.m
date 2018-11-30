% SSI data processing

%% load data
SSI_data = load("Ke3_Kv015.txt"); % Kv = 0.15 A/rad, alpha = 1*Kv, beta = 0.5*Kv
noSSI_data = load("Ke3.txt"); % Ke = 3 A/rad

% data format: [x*1000, f*1000, fe*1000, Id*1000, U1, Kdisp*1000]
% note: if f_SSI>fe, f = fe

%% scale data
SSI_data(:,1:4) = SSI_data(:,1:4)/1000; 
SSI_data(:,6) = SSI_data(:,6)/1000; 
noSSI_data(:,1:4) = noSSI_data(:,1:4)/1000;
noSSI_data(:,6) = noSSI_data(:,6)/1000; 

Imax = 1.5; % A
kt = 19.4; % in mNm/A
SSI_data(:,5) = ((SSI_data(:,5)-5000)/4000)*Imax*kt; % commanded T in mNm
noSSI_data(:,5) = ((noSSI_data(:,5)-5000)/4000)*Imax*kt;

%% plot things

figure; subplot(1,2,1); hold on;
plot(SSI_data(1:11100,1));
plot(noSSI_data(1:4750,1)); 
hold off;
subplot(1,2,2); hold on;
plot(SSI_data(1:11100,5));
plot(SSI_data(1:11100,2)*kt);
plot(noSSI_data(1:4750,5));
plot(noSSI_data(1:4750,3)*kt);
hold off;

figure; subplot(1,2,1); hold on;
plot(SSI_data(1:11100,1),SSI_data(1:11100,5));
plot(noSSI_data(1:4750,1),noSSI_data(1:4750,5));
hold off;
subplot(1,2,2); hold on;
plot(SSI_data(1:11100,5)./SSI_data(1:11100,1));
plot(noSSI_data(1:4750,5)./noSSI_data(1:4750,1));
hold off;

% figure;
% plot(noSSI_data(:,1),noSSI_data(:,3));

