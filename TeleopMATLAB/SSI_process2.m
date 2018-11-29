% SSI data processing

%% load data
SSI_data = load("SSI_Kv0_15_Ke3.txt"); % Kv = 0.15 A/rad, alpha = 1*Kv, beta = 0.5*Kv
noSSI_data = load("noSSI_Ke3.txt"); % Ke = 3 A/rad

% data format: [x*1000, f*1000, Id*1000, U1, Kdisp*1000]
% note: if f_SSI>fe, f = fe

%% scale data
SSI_data(:,1:3) = SSI_data(:,1:3)/1000; 
SSI_data(:,5) = SSI_data(:,5)/1000; 
noSSI_data(:,1:3) = noSSI_data(:,1:3)/1000;
noSSI_data(:,5) = noSSI_data(:,5)/1000; 

Imax = 1.5; % A
kt = 19.4; % in mNm/A
SSI_data(:,4) = ((SSI_data(:,2)-5000)/4000)*Imax*kt; % commanded T in mNm
noSSI_data(:,4) = ((noSSI_data(:,2)-5000)/4000)*Imax*kt;

%% plot things

figure;
plot(SSI_data(:,1),SSI_data(:,2));
figure;
plot(SSI_data(:,5));

figure;
plot(noSSI_data(:,1),noSSI_data(:,2));
figure;
plot(noSSI_data(:,5));

