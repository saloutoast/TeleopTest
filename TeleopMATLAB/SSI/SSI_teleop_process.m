% SSI virtual environment data processing

%% load data
SSI = load("SSItest_SSI.txt"); 
SSI_BD = load("SSItest_SSI_KdBonus.txt");
PD = load("SSItest_PD.txt"); 
P = load("SSItest_P.txt");
P_BD = load("SSItest_P_KdBonus.txt");

% data format: [SSI_case, U0-5000, (Pos1raw*(2*M_PI/8192)*1000), (ScaledPosDiff*1000), (ScaledVelDiff*1000), ((Ke+delO)*1000), (Id_SSI*1000), (Id_PD*1000)]

%% scale data
SSI(:,3:8) = SSI(:,3:8)/1000; 
SSI_BD(:,3:8) = SSI_BD(:,3:8)/1000;
PD(:,3:8) = PD(:,3:8)/1000;
P(:,3:8) = P(:,3:8)/1000;
P_BD(:,3:8) = P_BD(:,3:8)/1000;

Imax = 1.5; % A
kt = 19.4; % in mNm/A
SSI(:,2) = (SSI(:,2)/4000)*Imax*kt; % commanded T in mNm
SSI_BD(:,2) = (SSI_BD(:,2)/4000)*Imax*kt;
PD(:,2) = (PD(:,2)/4000)*Imax*kt;
P(:,2) = (P(:,2)/4000)*Imax*kt;
P_BD(:,2) = (P_BD(:,2)/4000)*Imax*kt;

% new data format: [SSI_case, actual current, Pos1, PosDiff, VelDiff, Ke+delO, Id_SSI, Id_PD]

%% plot the plots

figure; hold on;
plot(SSI(:,3));
plot(SSI(:,3)-SSI(:,4));
hold off;
title('SSI');

figure; hold on;
plot(SSI_BD(:,3));
plot(SSI_BD(:,3)-SSI_BD(:,4));
hold off;
title('SSI BD');

figure; hold on;
plot(PD(:,3));
plot(PD(:,3)-PD(:,4));
hold off;
title('PD');

figure; hold on;
plot(P(:,3));
plot(P(:,3)-P(:,4));
hold off;
title('P');

figure; hold on;
plot(P_BD(:,3));
plot(P_BD(:,3)-P_BD(:,4));
hold off;
title('P BD');

figure; plot(SSI(:,4)); title('SSI pos diff');
figure; plot(SSI_BD(:,4)); title('SSI BD pos diff');
figure; plot(P_BD(:,4)); title('P BD pos diff');

% figure; hold on;
% plot(SSI(:,2));
% plot(PD(:,2));
% hold off;