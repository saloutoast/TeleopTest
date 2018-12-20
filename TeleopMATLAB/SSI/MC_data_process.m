
%% open data from Logs folder
test_data = importdata("../../3DOF_controller/Logs/133104_12_20_2018.log");

%% split data to two cases
tPD = 0;
tSSI = 0;
for ii = 1:size(test_data,1)
    if (test_data(ii,1)==1)
        PD_data(ii,:) = [tPD, test_data(ii,2)/100, test_data(ii,3:end)/1000];
        tPD = tPD + 0.001;
    elseif (test_data(ii,1)==3)
        SSI_data(ii,:) = [tSSI, test_data(ii,2)/100, test_data(ii,3:end)/1000];
        tSSI = tSSI + 0.001;
    end        
end

%% split further?

