% SSI virtual environment data processing

%% load data
SSI = load("SSItest_SSI_Ke5.txt"); 
SSI_BD = load("SSItest_SSI_Ke5_Kbd01.txt");
PD = load("SSItest_PD_Kp5_Kd01.txt"); 
P = load("SSItest_P_Kp5.txt");
P_BD = load("SSItest_P_Kp5_Kbd01.txt");
% data format: [SSI_case, U0-5000, (Pos1raw*(2*M_PI/8192)*1000), (ScaledPosDiff*1000), (ScaledVelDiff*1000), (delO*1000), (Id_SSI*1000), (Id_PD*1000)]

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

% correct 2pi wrapping
for ii=1:size(SSI,1)
    if (SSI(ii,3)<1.0)
        SSI(ii,3) = SSI(ii,3) + 2*pi;
    end
end
for ii=1:size(SSI_BD,1)
    if (SSI_BD(ii,3)<1.0)
        SSI_BD(ii,3) = SSI_BD(ii,3) + 2*pi;
    end
end
for ii=1:size(PD,1)
    if (PD(ii,3)<1.0)
        PD(ii,3) = PD(ii,3) + 2*pi;
    end
end
for ii=1:size(P,1)
    if (P(ii,3)<1.0)
        P(ii,3) = P(ii,3) + 2*pi;
    end
end
for ii=1:size(P_BD,1)
    if (P_BD(ii,3)<1.0)
        P_BD(ii,3) = P_BD(ii,3) + 2*pi;
    end
end


% new data format: [SSI_case, actual current, Pos1, PosDiff, VelDiff, delO, Id_SSI, Id_PD]

%% get some statistics

RASE = zeros(1,5); % [P, PD, P_BD, SSI, SSI_BD]
RASE(1) = norm(P(:,4));
RASE(2) = norm(PD(:,4));
RASE(3) = norm(P_BD(:,4));
RASE(4) = norm(SSI(:,4));
RASE(5) = norm(SSI_BD(:,4));

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

% figure; hold on; 
% plot(SSI(:,4)); 
% plot(SSI_BD(:,4));
% plot(P_BD(:,4));
% hold off; legend('SSI','SSI BD','PD BD');
