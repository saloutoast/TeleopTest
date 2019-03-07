% train several one-class SVMs on collected data from mini-cheetah arms


%% load data

fprintf('Loading data...');

free_test_raw = importdata("../../../3DOF_controller/Logs/123259_03_06_2019.log");
col_test_raw = importdata("../../../3DOF_controller/Logs/123726_03_06_2019.log");

% data format: row A: scaling*100, q1[0:3]*1000, q2[0:3]*1000
% data format: row B: scaling*200, dq1[0:3]*1000, dq2[0:3]*1000

% new format: time, q1[0:3], q2[0:3], dq1[0:3], dq1[0:3]

% free motion data
step = 0.001;
time = 0.001;
data_len = size(free_test_raw,1);
free_data = zeros(data_len,13);
free_data(1,:) = [0, 0.001*free_test_raw(1,2:7), 0.001*free_test_raw(2,2:7)];

for ii=2:(data_len-1)
    
    if (free_test_raw(ii,1)==50) % position data
        free_data(ii,:) = [time, 0.001*free_test_raw(ii,2:7), (0.0005*(free_test_raw(ii+1,2:7)+free_test_raw(ii-1,2:7)))];
        time = time + step;
    elseif (free_test_raw(ii,1)==100 || free_test_raw(ii,1)==101)
        free_data(ii,:) = [time, (0.0005*(free_test_raw(ii+1,2:7)+free_test_raw(ii-1,2:7))), 0.001*free_test_raw(ii,2:7)];
        time = time + step;
    else
        fprintf('Bad step free %d at ii = %d \n', free_test_raw(ii,1), ii)    
    end
end

free_data(data_len,:) = [time, 0.001*free_test_raw(data_len-1,2:7), 0.001*free_test_raw(data_len,2:7)];

% collision motion data
step = 0.001;
time = 0.001;
data_len = size(col_test_raw,1);
col_data = zeros(data_len,13);
col_data(1,:) = [0, 0.001*col_test_raw(1,2:7), 0.001*col_test_raw(2,2:7)];
for ii=2:(data_len-1)
    
    if (col_test_raw(ii,1)==50) % position data
        col_data(ii,:) = [time, 0.001*col_test_raw(ii,2:7), (0.0005*(col_test_raw(ii+1,2:7)+col_test_raw(ii-1,2:7)))];
        time = time + step;
    elseif (col_test_raw(ii,1)==100 || col_test_raw(ii,1)==101)
        col_data(ii,:) = [time, (0.0005*(col_test_raw(ii+1,2:7)+col_test_raw(ii-1,2:7))), 0.001*col_test_raw(ii,2:7)];
        time = time + step;
    else
        fprintf('Bad step col at ii = %d \n', ii)    
    end
end
col_data(data_len,:) = [time, 0.001*col_test_raw(data_len-1,2:7), 0.001*col_test_raw(data_len,2:7)];

fprintf('done!\n');

%% calculate torque commands

Kp = 100.0;
Kd = 0.8;

% tau_g1 = 0.2264f*cosf(q1[1]) + 0.0253f*cosf(q1[1]+q1[2]); 
% tau_g2 = 0.2264f*cosf(q2[1]) + 0.0253f*cosf(q2[1]+q2[2]);
% 
% 
% tau1[0] = -scaling*(100.0f*(q2[0] - q1[0]) + 0.8f*(dq2[0] - dq1[0]));
% tau2[0] = -scaling*(100.0f*(q1[0] - q2[0]) + 0.8f*(dq1[0] - dq2[0]));
% 
% tau1[1] = scaling*(kp_q*(q2[1] - q1[1]) + kd_q*(dq2[1] - dq1[1])) + tau_g1;
% tau2[1] = scaling*(kp_q*(q1[1] - q2[1]) + kd_q*(dq1[1] - dq2[1])) + tau_g2;
% 
% tau1[2] = scaling*((kp_q/1.5f)*(q2[2] - q1[2]) + (kd_q/2.25f)*(dq2[2] - dq1[2]));
% tau2[2] = scaling*((kp_q/1.5f)*(q1[2] - q2[2]) + (kd_q/2.25f)*(dq1[2] - dq2[2]));

free_tau1 = zeros(length(free_data(:,1)),3);
free_tau2 = zeros(length(free_data(:,1)),3);

free_tau1(:,1) = (Kp/2)*(free_data(:,2)-free_data(:,5)) + (Kd/2)*(free_data(:,8)-free_data(:,11));
free_tau2(:,1) = -free_tau1(:,1);

free_tau1(:,2) = Kp*(free_data(:,6)-free_data(:,3)) + Kd*(free_data(:,12)-free_data(:,9)) + 0.2264*cos(free_data(:,3)) + 0.0253*cos(free_data(:,3)+free_data(:,4));
free_tau2(:,2) = Kp*(free_data(:,3)-free_data(:,6)) + Kd*(free_data(:,9)-free_data(:,12)) + 0.2264*cos(free_data(:,6)) + 0.0253*cos(free_data(:,6)+free_data(:,7));

free_tau1(:,3) = (Kp/1.5)*(free_data(:,7)-free_data(:,4)) + (Kd/2.25)*(free_data(:,13)-free_data(:,10));
free_tau2(:,3) = -free_tau1(:,3);

col_tau1 = zeros(length(col_data(:,1)),3);
col_tau2 = zeros(length(col_data(:,1)),3);

col_tau1(:,1) = (Kp/2)*(col_data(:,2)-col_data(:,5)) + (Kd/2)*(col_data(:,8)-col_data(:,11));
col_tau2(:,1) = -col_tau1(:,1);

col_tau1(:,2) = Kp*(col_data(:,6)-col_data(:,3)) + Kd*(col_data(:,12)-col_data(:,9)) + 0.2264*cos(col_data(:,3)) + 0.0253*cos(col_data(:,3)+col_data(:,4));
col_tau2(:,2) = Kp*(col_data(:,3)-col_data(:,6)) + Kd*(col_data(:,9)-col_data(:,12)) + 0.2264*cos(col_data(:,6)) + 0.0253*cos(col_data(:,6)+col_data(:,7));

col_tau1(:,3) = (Kp/1.5)*(col_data(:,7)-col_data(:,4)) + (Kd/2.25)*(col_data(:,13)-col_data(:,10));
col_tau2(:,3) = -col_tau1(:,3);


%% world frame endpoint position, velocity, force of slave

Labad = 0.0577;
Lhip = 0.2088;
Lknee = 0.175;

% for free motion data
s1 = sin(free_data(:,2));
s2 = sin(free_data(:,3));
s3 = sin(free_data(:,4));

c1 = cos(free_data(:,2));
c2 = cos(free_data(:,3));
c3 = cos(free_data(:,4));
     
c23 = c2.*c3 - s2.*s3;
s23 = s2.*c3 + c2.*s3;
     
% End effector position of slave
free_pos_end1 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];

free_vel_end1 = [];
free_f_end1 = [];

for ii=1:1:size(free_pos_end1,1)
    J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
        Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
        Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
    
    free_vel_end1(ii,:) = (J*[free_data(ii,8);free_data(ii,9);free_data(ii,10)])';
    free_f_end1(ii,:) = (pinv(J')*[free_tau1(ii,1);free_tau1(ii,2);free_tau1(ii,3)])';
end

s1 = sin(free_data(:,5));
s2 = sin(free_data(:,6));
s3 = sin(free_data(:,7));

c1 = cos(free_data(:,5));
c2 = cos(free_data(:,6));
c3 = cos(free_data(:,7));
     
c23 = c2.*c3 - s2.*s3;
s23 = s2.*c3 + c2.*s3;
     
% End effector position of slave
free_pos_end2 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];

free_vel_end2 = [];
free_f_end2 = [];

for ii=1:1:size(free_pos_end2,1)
    J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
        Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
        Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
    
    free_vel_end2(ii,:) = (J*[free_data(ii,11);free_data(ii,12);free_data(ii,13)])';
    free_f_end2(ii,:) = (pinv(J')*[free_tau2(ii,1);free_tau2(ii,2);free_tau2(ii,3)])';
end

% for collision data
s1 = sin(col_data(:,2));
s2 = sin(col_data(:,3));
s3 = sin(col_data(:,4));

c1 = cos(col_data(:,2));
c2 = cos(col_data(:,3));
c3 = cos(col_data(:,4));
     
c23 = c2.*c3 - s2.*s3;
s23 = s2.*c3 + c2.*s3;
     
% End effector position of slave
col_pos_end1 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];

col_vel_end1 = [];
col_f_end1 = [];

for ii=1:1:size(col_pos_end1,1)
    J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
        Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
        Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
    
    col_vel_end1(ii,:) = (J*[col_data(ii,8);col_data(ii,9);col_data(ii,10)])';
    col_f_end1(ii,:) = (pinv(J')*[col_tau1(ii,1);col_tau1(ii,2);col_tau1(ii,3)])';
end
 
s1 = sin(col_data(:,5));
s2 = sin(col_data(:,6));
s3 = sin(col_data(:,7));

c1 = cos(col_data(:,5));
c2 = cos(col_data(:,6));
c3 = cos(col_data(:,7));
     
c23 = c2.*c3 - s2.*s3;
s23 = s2.*c3 + c2.*s3;
     
% End effector position of slave
col_pos_end2 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];

col_vel_end2 = [];
col_f_end2 = [];

for ii=1:1:size(col_pos_end2,1)
    J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
        Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
        Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
    
    col_vel_end2(ii,:) = (J*[col_data(ii,11);col_data(ii,12);col_data(ii,13)])';
    col_f_end2(ii,:) = (pinv(J')*[col_tau2(ii,1);col_tau2(ii,2);col_tau2(ii,3)])';
end

%% train SVM(s)

% train for no outliers

bs_back = 10; %[3, 5, 10];
bs_for = 1; %[1, 3, 5];
outfracs = 0.2; %0.0:0.05:0.25;

for ii=1:length(bs_back)
    for jj=1:length(bs_for)
        for hh=1:length(outfracs)
            bucket_size_back = bs_back(ii);
            bucket_size_for = bs_for(jj);
            outfrac = outfracs(hh);  
    %             offset = offsets(hh);
            nu = 0.2;

            fprintf('Bucket size of %d back, %d forward, outlier fraction of %.3f\n', bucket_size_back, bucket_size_for, outfrac);

            
            
            X1 = [movmean(free_vel_end(:,1), [bucket_size_back,0]), movmean(free_vel_end(:,1), [0,bucket_size_for])];
            X1 = [X1; -10, 10];
            y1 = ones(size(X1,1),1);
            y1(end) = -1;


    %             X2 = zeros((size(X1,1)-2*(offset+bucket_size)),2);
    %             for kk = 1:(size(X1,1)-2*(offset+bucket_size))
    %                X2(ii,:) = [X1(kk+bucket_size,1), X1(kk+offset+offset+bucket_size,2)];
    %             end
    %             y2 = ones(size(X2,1),1);

    %             k = randperm(data_len-bucket_size);
    %             Xsamp = X1(k(1:1000),:);

    %         fprintf('Training SVM...');
            SVMModel = fitcsvm(X1,y1,'KernelFunction','linear','Standardize',false); %,'Nu',nu); %...
                %'OutlierFraction',outfrac);
    %         fprintf('done!\n');

            mmback_zvel = movmean(col_vel_end(:,1), [bucket_size_back, 0]);
            mmfor_zvel = movmean(col_vel_end(:,1), [0, bucket_size_for]);

    %             for kk=1:(size(col_data,1)-2*(offset+bucket_size))
    %                pred_data(kk,:) = [mmback_zvel(kk+bucket_size), mmfor_zvel(kk+(2*offset)+bucket_size)];
    %             end

            pred_data = [mmback_zvel, mmfor_zvel];

            [labels,scores] = predict(SVMModel,pred_data);
%             scores_bool = (scores < 0); % for kernel
            scores_bool = (scores(:,2) < 1); % for linear

            x1test = -1:0.01:1;
            x2test = (1/SVMModel.Beta(2))*(1-SVMModel.Bias-SVMModel.Beta(1)*x1test);
            
            
            figure; hold on; ylim([-1,1]); xlim([-1,1]);
            plot(X1(:,1),X1(:,2));
            plot(pred_data(:,1),pred_data(:,2));
            plot(x1test,x2test);
            
            fig = figure;
            plot(col_data(:,1), col_pos_end(:,1), col_data(:,1), scores_bool);
            tl = sprintf('No stand., b.s.b. =  %d, b.s.f. = %d, nu = %.1f and o.f. = %.3f', bucket_size_back, bucket_size_for, nu, outfrac);
            title(tl)
%             filename = fullfile(pwd, 'Figures', sprintf('SVM_zvel_b%d_f%d_%d',bucket_size_back,bucket_size_for,(outfrac*1000)));
%             print(fig, filename,'-dpng');
%             close(fig);
        end 
    end
end

%% simple classifier based on pos diff and vel diff (cartesian space...should easily generalize to x,y, unlike if they were joint limits)

pos_lim = 0:0.0001:0.0025;
vel_lim = 0:0.01:0.4;

col_bools_p = zeros(length(col_data(:,1)),length(pos_lim));
col_bools_v = zeros(length(col_data(:,1)),length(vel_lim));

for ii=1:length(pos_lim)
    plim = pos_lim(ii);        
    col_bools_p(:,ii) = abs(col_pos_end1(:,1)-col_pos_end2(:,1))>plim;
    
    
    fig = figure;
    plot(col_data(:,1), col_pos_end2(:,1), col_data(:,1), col_bools_p(:,ii));
    tl = sprintf('Plim: %.4f', pos_lim(ii));
    title(tl)
    
    filename = fullfile(pwd, 'Figures', sprintf('BoolClass_zvel_P%d',ii));
    print(fig, filename,'-dpng');
    close(fig);
    
end

for ii=1:length(vel_lim)
    vlim = vel_lim(ii);
    col_bools_v(:,ii) = abs(col_vel_end1(:,1)-col_vel_end2(:,1))>vlim;
    
    fig = figure;
    plot(col_data(:,1), col_pos_end2(:,1), col_data(:,1), col_bools_v(:,ii));
    tl = sprintf('Vlim: %.2f', vel_lim(ii));
    title(tl)
    
    filename = fullfile(pwd, 'Figures', sprintf('BoolClass_zvel_V%d',ii));
    print(fig, filename,'-dpng');
    close(fig);
end

% for ii=1:length(pos_lim)
%     for jj=1:length(vel_lim)
%                 
%         fprintf('Plim: %.4d, Vlim: %.2f\n', pos_lim(ii), vel_lim(jj));
%         
%         fig = figure;
%         subplot(3,1,1);
%         hold on;
%         plot(col_data(:,1), col_pos_end2(:,1), col_data(:,1), col_bools_p(:,ii));
%         tl = sprintf('Plim: %.4f, Vlim: %.2f', pos_lim(ii), vel_lim(jj));
%         title(tl)
%         subplot(3,1,2);
%         plot(col_data(:,1), col_pos_end2(:,1),col_data(:,1), col_bools_v(:,jj));
%         subplot(3,1,3);
%         plot(col_data(:,1), col_pos_end2(:,1),col_data(:,1), (col_bools_p(:,ii)&col_bools_v(:,jj)));
% 
% 
%         filename = fullfile(pwd, 'Figures', sprintf('BoolClass_zvel_P%d_V%d',ii, jj));
%         print(fig, filename,'-dpng');
%         close(fig);
%         
% %         pause
%         
%         
%     end
% end














