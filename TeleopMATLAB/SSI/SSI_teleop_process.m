% SSI virtual environment data processing

%% load data
SSI_data = load("SSI_teleop_test1.txt"); 
PD_data = load("SSI_teleop_test2_PD.txt"); 

% data format: [SSI_case, U0-5000, (Pos1raw*(2*M_PI/8192)*1000), (ScaledPosDiff*1000), (ScaledVelDiff*1000), ((Ke+delO)*1000), (Id_SSI*1000), (Id_PD*1000)]

%% scale data
SSI_data(:,3:8) = SSI_data(:,3:8)/1000; 
PD_data(:,3:8) = PD_data(:,3:8)/1000;

Imax = 1.5; % A
kt = 19.4; % in mNm/A
SSI_data(:,2) = (SSI_data(:,2)/4000)*Imax*kt; % commanded T in mNm
PD_data(:,2) = (PD_data(:,2)/4000)*Imax*kt;

% new data format: [SSI_case, actual current, Pos1, PosDiff, VelDiff, Ke+delO, Id_SSI, Id_PD]

%% plot the plots

figure; hold on;
plot(SSI_data(:,3));
plot(SSI_data(:,3)-SSI_data(:,4));
hold off;

figure; hold on;
plot(PD_data(:,3));
plot(PD_data(:,3)-PD_data(:,4));
hold off;

figure; hold on;
plot(SSI_data(:,2));
plot(PD_data(:,2));
hold off;