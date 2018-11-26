% SSI data processing

%% load data
SSI_data = load("SSI_test2.txt");
noSSI_data = load("noSSI_test2.txt");

%% scale data
SSI_data(:,1) = SSI_data(:,1)/1000; 
noSSI_data(:,1) = noSSI_data(:,1)/1000;

Imax = 1.5; % A
kt = 19.4; % in mNm/A
SSI_data(:,2) = ((SSI_data(:,2)-5000)/4000)*Imax*kt; % commanded T in mNm
noSSI_data(:,2) = ((noSSI_data(:,2)-5000)/4000)*Imax*kt;

SSI_data(:,4:5) = SSI_data(:,4:5)*(kt/1000); % SSI and VE T in mNm
noSSI_data(:,4:5) = noSSI_data(:,4:5)*(kt/1000);

%% plot something

figure;
plot(noSSI_data(:,1),noSSI_data(:,2)); %./noSSI_data(:,1));

figure;
plot(SSI_data(:,1),SSI_data(:,2)); %./SSI_data(:,1));