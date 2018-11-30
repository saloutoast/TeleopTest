% SSI data processing

%% load data
SSI_data = load("Ke3_Kv015.txt"); % Kv = 0.15 A/rad, alpha = 1*Kv, beta = 0.5*Kv, note: if f_SSI>fe, f = fe
noSSI_data = load("Ke3.txt"); % Ke = 3 A/rad

noSSI_data_2 = load("Ke1000.txt"); % Ke = 1000 A/rad
SSI_data_2 = load("Ke1000_Kv1.txt"); % Ke = 1000, Kv = 1, note: if f_SSI>fe, f = fe
SSI_data_3 = load("Ke1000_Kv1_nocap.txt"); % Ke = 1000, Kv = 1, no cap on SSI stiffness

% data format: [x*1000, f*1000, fe*1000, Id*1000, U1, Kdisp*1000]

%% scale data
SSI_data(:,1:4) = SSI_data(:,1:4)/1000; 
SSI_data(:,6) = SSI_data(:,6)/1000; 
noSSI_data(:,1:4) = noSSI_data(:,1:4)/1000;
noSSI_data(:,6) = noSSI_data(:,6)/1000; 

SSI_data_2(:,1:4) = SSI_data_2(:,1:4)/1000; 
SSI_data_2(:,6) = SSI_data_2(:,6)/1000; 
SSI_data_3(:,1:4) = SSI_data_3(:,1:4)/1000; 
SSI_data_3(:,6) = SSI_data_3(:,6)/1000; 
noSSI_data_2(:,1:4) = noSSI_data_2(:,1:4)/1000;
noSSI_data_2(:,6) = noSSI_data_2(:,6)/1000; 

Imax = 1.5; % A
kt = 19.4; % in mNm/A
SSI_data(:,5) = ((SSI_data(:,5)-5000)/4000)*Imax*kt; % commanded T in mNm
noSSI_data(:,5) = ((noSSI_data(:,5)-5000)/4000)*Imax*kt;

SSI_data_2(:,5) = ((SSI_data_2(:,5)-5000)/4000)*Imax*kt;
SSI_data_3(:,5) = ((SSI_data_3(:,5)-5000)/4000)*Imax*kt;
noSSI_data_2(:,5) = ((noSSI_data_2(:,5)-5000)/4000)*Imax*kt;

%% plot the plots

% figure; subplot(1,2,1); hold on;
% plot(SSI_data(1:11100,1));
% plot(noSSI_data(1:4750,1)); 
% hold off;
% subplot(1,2,2); hold on;
% plot(SSI_data(1:11100,5));
% plot(SSI_data(1:11100,2)*kt);
% plot(noSSI_data(1:4750,5));
% plot(noSSI_data(1:4750,3)*kt);
% hold off;
% 
% figure; subplot(1,2,1); hold on;
% plot(SSI_data(1:11100,1),SSI_data(1:11100,5));
% plot(noSSI_data(1:4750,1),noSSI_data(1:4750,5));
% hold off;
% subplot(1,2,2); hold on;
% plot(SSI_data(1:11100,5)./SSI_data(1:11100,1));
% plot(noSSI_data(1:4750,5)./noSSI_data(1:4750,1));
% hold off;

figure; subplot(1,2,1); hold on;
plot(noSSI_data_2(:,1)); 
plot(SSI_data_2(1:10600,1));
plot(SSI_data_3(1:10600,1),'g');
hold off;
subplot(1,2,2); hold on;
plot(noSSI_data_2(:,5));
% plot(noSSI_data_2(:,3)*kt);
plot(SSI_data_2(1:10600,5));
% plot(SSI_data_2(1:10600,2)*kt);
plot(SSI_data_3(1:10600,5),'g');
% plot(SSI_data_3(1:10600,2)*kt);
hold off; legend('noSSI','SSI2','SSI3');

figure; subplot(1,2,1); hold on;
plot(noSSI_data_2(:,1),noSSI_data_2(:,5),'LineWidth',1.5);
plot(SSI_data_2(1:10600,1),SSI_data_2(1:10600,5));
plot(SSI_data_3(1:10600,1),SSI_data_3(1:10600,5),'g');
hold off;
subplot(1,2,2); hold on;
plot(noSSI_data_2(:,5)./noSSI_data_2(:,1));
plot(SSI_data_2(1:10600,5)./SSI_data_2(1:10600,1));
plot(SSI_data_3(1:10600,5)./SSI_data_3(1:10600,1),'g');
hold off; legend('noSSI','SSI2','SSI3'); ylim([0, 5000]);

