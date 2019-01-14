% chirp testing data processing

%% load data

P_data = load("chirptest_P_Kp5.txt");
PD_data = load("chirptest_PD_Kp5_Kd01.txt");
SSI2_data = load("chirptest_SSI_2way_Ke20.txt");
SSI_data = load("chirptest_SSI_Ke20.txt");
SSIbd_data = load("chirptest_SSI_Ke20_Kbd01.txt");

% format data

time = 0:0.001:48;

% format: [omega, pos1, posdiff, veldiff, (delO)]
P_data = [P_data(:,4), P_data(:,1:3)]/1000;
PD_data = [PD_data(:,4), PD_data(:,1:3)]/1000;
SSI_data = SSI_data/1000;
SSI2_data = SSI2_data/1000;
SSIbd_data = SSIbd_data/1000;

%% get statistics
% statistics [MSE, max err, min err] x [P, PD, SSI, SSIbd, SSI2]
stats = [norm(P_data(:,3))/length(P_data(:,3)), norm(PD_data(:,3))/length(PD_data(:,3)), norm(SSI_data(:,3))/length(SSI_data(:,3)), norm(SSIbd_data(:,3))/length(SSIbd_data(:,3)), norm(SSI2_data(:,3))/length(SSI2_data(:,3));
         max(P_data(:,3)), max(PD_data(:,3)), max(SSI_data(:,3)), max(SSIbd_data(:,3)), max(SSI2_data(:,3));
         min(P_data(:,3)), min(PD_data(:,3)), min(SSI_data(:,3)), min(SSIbd_data(:,3)), min(SSI2_data(:,3))];

%% plot positions

figure;

to_plot = SSIbd_data;
subplot(2,1,2); hold on;
plot(time(20000:22000),to_plot(20000:22000,2));
plot(time(20000:22000),to_plot(20000:22000,2)-to_plot(20000:22000,3));
hold off; ylim([2.5, 4]);
title("SSI w/ bonus damping"); legend('Master','Slave');

to_plot = PD_data;
subplot(2,1,1); hold on;
plot(time(20000:22000),to_plot(20000:22000,2));
plot(time(20000:22000),to_plot(20000:22000,2)-to_plot(20000:22000,3));
hold off; ylim([2.5, 4]);
title("PD"); legend('Master','Slave');

% to_plot = P_data;
% subplot(3,1,1); hold on;
% plot(time(1:size(to_plot,1)),to_plot(:,2));
% plot(time(1:size(to_plot,1)),to_plot(:,2)-to_plot(:,3));
% hold off; ylim([2.5, 4]);
% title("P only"); legend('Master','Slave');

% figure; hold on;
% plot(time(1:size(P_data,1)),P_data(:,3));
% plot(time(1:size(PD_data,1)),PD_data(:,3));
% plot(time(1:size(SSIbd_data,1)),SSIbd_data(:,3));
% hold off; title("Pos Errors"); legend('P only','PD','SSI w/ BD');

%% plot freq response of P, PD, and SSI

Fs = 1000; % sampling frequency
T = 1/Fs; % sampling period

to_fft = P_data;
L = size(to_fft,1);
t = (0:L-1)*T; % time vector
F = fft(to_fft(:,3)); % fft of position error
P2 = abs(F/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1); % frequency response for plotting
f = Fs*(0:(L/2))/L;

figure;
plot(f(1:2000),P1(1:2000));
title("Single-Sided Amplitude Spectrum of PosError for P");
xlabel('f'); ylabel('|P1|');

to_fft = PD_data;
L = size(to_fft,1);
t = (0:L-1)*T; % time vector
F = fft(to_fft(:,3)); % fft of position error
P2 = abs(F/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1); % frequency response for plotting
f = Fs*(0:(L/2))/L;

figure;
plot(f(1:2000),P1(1:2000));
title("Single-Sided Amplitude Spectrum of PosError for PD");
xlabel('f'); ylabel('|P1|');

to_fft = SSI_data;
L = size(to_fft,1);
t = (0:L-1)*T; % time vector
F = fft(to_fft(:,3)); % fft of position error
P2 = abs(F/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1); % frequency response for plotting
f = Fs*(0:(L/2))/L;

figure;
plot(f(1:2000),P1(1:2000));
title("Single-Sided Amplitude Spectrum of PosError for SSI");
xlabel('f'); ylabel('|P1|');

% bode plots from estimate of transfer function between master position and slave position
figure;
tfestimate(P_data(:,2),(P_data(:,2)-P_data(:,3)),[],[],[],1000);
xlim([0 25]);
figure;
tfestimate(PD_data(:,2),(PD_data(:,2)-PD_data(:,3)),[],[],[],1000);
xlim([0 25]);
figure;
tfestimate(SSIbd_data(:,2),(SSIbd_data(:,2)-SSIbd_data(:,3)),[],[],[],1000);
xlim([0 25]);

