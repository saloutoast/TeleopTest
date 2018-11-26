% plotting data from prelim testing of 1-stage belt reduction on 1DOF
% teleoperation apparatus

%% import data
data1 = load('reduction_contact2.txt');
data2 = load('reduction_contact3.txt');

%% parse

Kp1 = data1(1001:8000,1)';
Id1 = data1(1001:8000,2)';
PosDiff1 = data1(1001:8000,3)';
VelDiff1 = data1(1001:8000,4)';
Contact1 = (data1(1001:8000,5)' < 3000);

Kp2 = data2(1:7000,1)';
Id2 = data2(1:7000,2)';
PosDiff2 = data2(1:7000,3)';
VelDiff2 = data2(1:7000,4)';
Contact2 = (data2(1:7000,5)' < 3000);

%% plot values

time = 0.001:0.001:7;

figure; 
subplot(2,1,1);
hold on;
plot(time,VelDiff1);
plot(time,(Contact1*50));
plot(time,(Kp1/20));
plot(time,(PosDiff1/100));
hold off;
subplot(2,1,2);
hold on;
plot(time,VelDiff2);
plot(time,(Contact2*50));
plot(time,(Kp2/20));
plot(time,(PosDiff2/100));
hold off;


