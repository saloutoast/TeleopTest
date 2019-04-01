% train several one-class SVMs on collected data from mini-cheetah arms


%% load data

fprintf('Loading data...');

free_test_raw = importdata("../../../3DOF_controller/Logs/123259_03_06_2019.log");
% col_test_raw = importdata("../../../3DOF_controller/Logs/123726_03_06_2019.log"); % old z collision data
% col_test_raw = importdata("../../../3DOF_controller/Logs/135224_03_07_2019.log"); % new z collision data
% col_test_raw = importdata("../../../3DOF_controller/Logs/130046_03_08_2019.log"); % new z, xy collision data
% col_test_raw = importdata("../../../3DOF_controller/Logs/134738_03_08_2019.log"); % testing direction of impact forces on master
col_test_z10 = importdata("../../../3DOF_controller/Logs/181001_03_13_2019.log");
col_test_x10 = importdata("../../../3DOF_controller/Logs/181055_03_13_2019.log");
col_test_z15 = importdata("../../../3DOF_controller/Logs/181358_03_13_2019.log");

col_test_new = importdata("../../../3DOF_controller/Logs/095437_03_14_2019.log");

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
col_test_raw = col_test_new;
step = 0.001;
time = 0.001;
data_len = size(col_test_raw,1);
col_data = zeros(data_len,13);
col_data(1,:) = [0, 0.001*col_test_raw(1,2:7), 0.001*col_test_raw(2,2:7)];
col_detection_z10 = zeros(data_len,4);
for ii=2:(data_len-1)
    
    if (col_test_raw(ii,1)==50) % position data
        col_data(ii,:) = [time, 0.001*col_test_raw(ii,2:7), (0.0005*(col_test_raw(ii+1,2:7)+col_test_raw(ii-1,2:7)))];
        time = time + step;
    else % (col_test_raw(ii,1)==100 || col_test_raw(ii,1)==101)
        col_data(ii,:) = [time, (0.0005*(col_test_raw(ii+1,2:7)+col_test_raw(ii-1,2:7))), 0.001*col_test_raw(ii,2:7)];
        
        col_pack = num2str(col_test_raw(ii,1)); % turn pack into string for indexing
        % append zeros to the front for indexing
        if (length(col_pack)==1)
            col_pack = ['000',col_pack];
        elseif (length(col_pack)==2)
            col_pack = ['00',col_pack];
        elseif (length(col_pack)==3)
            col_pack = ['0',col_pack];
        end
        col_detection(ii,:) = [str2num(col_pack(1)), str2num(col_pack(2)), str2num(col_pack(3)), str2num(col_pack(4))];
        col_detection(ii+1,:) = [str2num(col_pack(1)), str2num(col_pack(2)), str2num(col_pack(3)), str2num(col_pack(4))];
        
        time = time + step;
%     else
%         fprintf('Bad step col at ii = %d \n', ii)    
    end
end
col_data(data_len,:) = [time, 0.001*col_test_raw(data_len-1,2:7), 0.001*col_test_raw(data_len,2:7)];
col_data_new = col_data;
col_detection_new = col_detection;

% col_test_raw = col_test_x10;
% step = 0.001;
% time = 0.001;
% data_len = size(col_test_raw,1);
% col_data = zeros(data_len,13);
% col_data(1,:) = [0, 0.001*col_test_raw(1,2:7), 0.001*col_test_raw(2,2:7)];
% col_detection = zeros(data_len,4);
% for ii=2:(data_len-1)
%     
%     if (col_test_raw(ii,1)==50) % position data
%         col_data(ii,:) = [time, 0.001*col_test_raw(ii,2:7), (0.0005*(col_test_raw(ii+1,2:7)+col_test_raw(ii-1,2:7)))];
%         time = time + step;
%     else % (col_test_raw(ii,1)==100 || col_test_raw(ii,1)==101)
%         col_data(ii,:) = [time, (0.0005*(col_test_raw(ii+1,2:7)+col_test_raw(ii-1,2:7))), 0.001*col_test_raw(ii,2:7)];
%         
%         col_pack = num2str(col_test_raw(ii,1)); % turn pack into string for indexing
%         % append zeros to the front for indexing
%         if (length(col_pack)==1)
%             col_pack = ['000',col_pack];
%         elseif (length(col_pack)==2)
%             col_pack = ['00',col_pack];
%         elseif (length(col_pack)==3)
%             col_pack = ['0',col_pack];
%         end
%         col_detection(ii,:) = [str2num(col_pack(1)), str2num(col_pack(2)), str2num(col_pack(3)), str2num(col_pack(4))];
%         col_detection(ii+1,:) = [str2num(col_pack(1)), str2num(col_pack(2)), str2num(col_pack(3)), str2num(col_pack(4))];
%         
%         time = time + step;
% %     else
% %         fprintf('Bad step col at ii = %d \n', ii)    
%     end
% end
% col_data(data_len,:) = [time, 0.001*col_test_raw(data_len-1,2:7), 0.001*col_test_raw(data_len,2:7)];
% col_data_x10 = col_data;
% col_detection_x10 = col_detection;
% 
% col_test_raw = col_test_z15;
% step = 0.001;
% time = 0.001;
% data_len = size(col_test_raw,1);
% col_data = zeros(data_len,13);
% col_data(1,:) = [0, 0.001*col_test_raw(1,2:7), 0.001*col_test_raw(2,2:7)];
% col_detection = zeros(data_len,4);
% for ii=2:(data_len-1)
%     
%     if (col_test_raw(ii,1)==50) % position data
%         col_data(ii,:) = [time, 0.001*col_test_raw(ii,2:7), (0.0005*(col_test_raw(ii+1,2:7)+col_test_raw(ii-1,2:7)))];
%         time = time + step;
%     else % (col_test_raw(ii,1)==100 || col_test_raw(ii,1)==101)
%         col_data(ii,:) = [time, (0.0005*(col_test_raw(ii+1,2:7)+col_test_raw(ii-1,2:7))), 0.001*col_test_raw(ii,2:7)];
%         
%         col_pack = num2str(col_test_raw(ii,1)); % turn pack into string for indexing
%         % append zeros to the front for indexing
%         if (length(col_pack)==1)
%             col_pack = ['000',col_pack];
%         elseif (length(col_pack)==2)
%             col_pack = ['00',col_pack];
%         elseif (length(col_pack)==3)
%             col_pack = ['0',col_pack];
%         end
%         col_detection(ii,:) = [str2num(col_pack(1)), str2num(col_pack(2)), str2num(col_pack(3)), str2num(col_pack(4))];
%         col_detection(ii+1,:) = [str2num(col_pack(1)), str2num(col_pack(2)), str2num(col_pack(3)), str2num(col_pack(4))];
%         
%         time = time + step;
% %     else
% %         fprintf('Bad step col at ii = %d \n', ii)    
%     end
% end
% col_data(data_len,:) = [time, 0.001*col_test_raw(data_len-1,2:7), 0.001*col_test_raw(data_len,2:7)];
% col_data_z15 = col_data;
% col_detection_z15 = col_detection;

fprintf('done!\n');

%% calculate torque commands

fprintf('Calculating torques...');

Kp = 100.0;
Kd = 0.8;

free_tau1 = zeros(length(free_data(:,1)),3);
free_tau2 = zeros(length(free_data(:,1)),3);

free_tau1(:,1) = (Kp/2)*(free_data(:,2)-free_data(:,5)) + (Kd/2)*(free_data(:,8)-free_data(:,11));
free_tau2(:,1) = -free_tau1(:,1);

free_tau1(:,2) = Kp*(free_data(:,6)-free_data(:,3)) + Kd*(free_data(:,12)-free_data(:,9)) + 0.2264*cos(free_data(:,3)) + 0.0253*cos(free_data(:,3)+free_data(:,4));
free_tau2(:,2) = Kp*(free_data(:,3)-free_data(:,6)) + Kd*(free_data(:,9)-free_data(:,12)) + 0.2264*cos(free_data(:,6)) + 0.0253*cos(free_data(:,6)+free_data(:,7));

free_tau1(:,3) = (Kp/1.5)*(free_data(:,7)-free_data(:,4)) + (Kd/2.25)*(free_data(:,13)-free_data(:,10));
free_tau2(:,3) = -free_tau1(:,3);

col_tau1_new= zeros(length(col_data_new(:,1)),3);
col_tau2_new = zeros(length(col_data_new(:,1)),3);

col_tau1_new(:,1) = col_data_new(:,11);
col_tau2_new(:,1) = -(Kp/2)*(col_data_new(:,2)-col_data_new(:,5)) + (Kd/2)*(col_data_new(:,8));

col_tau1_new(:,2) = col_data_new(:,12); %Kp*(col_data_new(:,6)-col_data_new(:,3)) + Kd*(-col_data_new(:,9)) + 0.2264*cos(col_data_new(:,3)) + 0.0253*cos(col_data_new(:,3)+col_data_new(:,4));
col_tau2_new(:,2) = Kp*(col_data_new(:,3)-col_data_new(:,6)) + Kd*(col_data_new(:,9)) + 0.2264*cos(col_data_new(:,6)) + 0.0253*cos(col_data_new(:,6)+col_data_new(:,7));

col_tau1_new(:,3) = col_data_new(:,13);
col_tau2_new(:,3) = -(Kp/1.5)*(col_data_new(:,7)-col_data_new(:,4)) + (Kd/2.25)*(-col_data_new(:,10));


% col_tau1_x10 = zeros(length(col_data_x10(:,1)),3);
% col_tau2_x10 = zeros(length(col_data_x10(:,1)),3);
% 
% col_tau1_x10(:,1) = (Kp/2)*(col_data_x10(:,2)-col_data_x10(:,5)) + (Kd/2)*(col_data_x10(:,8)-col_data_x10(:,11));
% col_tau2_x10(:,1) = -col_tau1_x10(:,1);
% 
% col_tau1_x10(:,2) = Kp*(col_data_x10(:,6)-col_data_x10(:,3)) + Kd*(col_data_x10(:,12)-col_data_x10(:,9)) + 0.2264*cos(col_data_x10(:,3)) + 0.0253*cos(col_data_x10(:,3)+col_data_x10(:,4));
% col_tau2_x10(:,2) = Kp*(col_data_x10(:,3)-col_data_x10(:,6)) + Kd*(col_data_x10(:,9)-col_data_x10(:,12)) + 0.2264*cos(col_data_x10(:,6)) + 0.0253*cos(col_data_x10(:,6)+col_data_x10(:,7));
% 
% col_tau1_x10(:,3) = (Kp/1.5)*(col_data_x10(:,7)-col_data_x10(:,4)) + (Kd/2.25)*(col_data_x10(:,13)-col_data_x10(:,10));
% col_tau2_x10(:,3) = -col_tau1_x10(:,3);
% 
% 
% col_tau1_z15 = zeros(length(col_data_z15(:,1)),3);
% col_tau2_z15 = zeros(length(col_data_z15(:,1)),3);
% 
% col_tau1_z15(:,1) = (Kp/2)*(col_data_z15(:,2)-col_data_z15(:,5)) + (Kd/2)*(col_data_z15(:,8)-col_data_z15(:,11));
% col_tau2_z15(:,1) = -col_tau1_z15(:,1);
% 
% col_tau1_z15(:,2) = Kp*(col_data_z15(:,6)-col_data_z15(:,3)) + Kd*(col_data_z15(:,12)-col_data_z15(:,9)) + 0.2264*cos(col_data_z15(:,3)) + 0.0253*cos(col_data_z15(:,3)+col_data_z15(:,4));
% col_tau2_z15(:,2) = Kp*(col_data_z15(:,3)-col_data_z15(:,6)) + Kd*(col_data_z15(:,9)-col_data_z15(:,12)) + 0.2264*cos(col_data_z15(:,6)) + 0.0253*cos(col_data_z15(:,6)+col_data_z15(:,7));
% 
% col_tau1_z15(:,3) = (Kp/1.5)*(col_data_z15(:,7)-col_data_z15(:,4)) + (Kd/2.25)*(col_data_z15(:,13)-col_data_z15(:,10));
% col_tau2_z15(:,3) = -col_tau1_z15(:,3);

fprintf('done!\n');

%% world frame endpoint position, velocity, force of slave

fprintf('Calculating cartesian data...');

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
col_data = col_data_new;
s1 = sin(col_data(:,2));
s2 = sin(col_data(:,3));
s3 = sin(col_data(:,4));

c1 = cos(col_data(:,2));
c2 = cos(col_data(:,3));
c3 = cos(col_data(:,4));
     
c23 = c2.*c3 - s2.*s3;
s23 = s2.*c3 + c2.*s3;
     
% End effector position of master
col_pos_end1 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];

% % col_vel_end1 = [];
% col_f_end1 = [];
% % 
% for ii=1:1:size(col_pos_end1,1)
%     J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
%         Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
%         Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
%     
% %     col_vel_end1(ii,:) = (J*[col_data(ii,8);col_data(ii,9);col_data(ii,10)])';
%     col_f_end1(ii,:) = (pinv(J')*[col_tau1(ii,1);col_tau1(ii,2);col_tau1(ii,3)])';
% end
 
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

% col_vel_end2 = [];
% col_f_end2 = [];

% for ii=1:1:size(col_pos_end2,1)
%     J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
%         Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
%         Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
%     
%     col_vel_end2(ii,:) = (J*[col_data(ii,11);col_data(ii,12);col_data(ii,13)])';
% %     col_f_end2(ii,:) = (pinv(J')*[col_tau2(ii,1);col_tau2(ii,2);col_tau2(ii,3)])';
% end

col_pos_end1_new = col_pos_end1;
% col_vel_end1_z10 = col_vel_end1;
col_pos_end2_new = col_pos_end2;
% col_vel_end2_z10 = col_vel_end2;



% col_data = col_data_x10;
% s1 = sin(col_data(:,2));
% s2 = sin(col_data(:,3));
% s3 = sin(col_data(:,4));
% 
% c1 = cos(col_data(:,2));
% c2 = cos(col_data(:,3));
% c3 = cos(col_data(:,4));
%      
% c23 = c2.*c3 - s2.*s3;
% s23 = s2.*c3 + c2.*s3;
%      
% % End effector position of master
% col_pos_end1 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];
% 
% col_vel_end1 = [];
% % col_f_end1 = [];
% % 
% for ii=1:1:size(col_pos_end1,1)
%     J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
%         Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
%         Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
%     
%     col_vel_end1(ii,:) = (J*[col_data(ii,8);col_data(ii,9);col_data(ii,10)])';
% %     col_f_end1(ii,:) = (pinv(J')*[col_tau1(ii,1);col_tau1(ii,2);col_tau1(ii,3)])';
% end
%  
% s1 = sin(col_data(:,5));
% s2 = sin(col_data(:,6));
% s3 = sin(col_data(:,7));
% 
% c1 = cos(col_data(:,5));
% c2 = cos(col_data(:,6));
% c3 = cos(col_data(:,7));
%      
% c23 = c2.*c3 - s2.*s3;
% s23 = s2.*c3 + c2.*s3;
%      
% % End effector position of slave
% col_pos_end2 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];
% 
% col_vel_end2 = [];
% % col_f_end2 = [];
% 
% for ii=1:1:size(col_pos_end2,1)
%     J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
%         Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
%         Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
%     
%     col_vel_end2(ii,:) = (J*[col_data(ii,11);col_data(ii,12);col_data(ii,13)])';
% %     col_f_end2(ii,:) = (pinv(J')*[col_tau2(ii,1);col_tau2(ii,2);col_tau2(ii,3)])';
% end
% 
% col_pos_end1_x10 = col_pos_end1;
% col_vel_end1_x10 = col_vel_end1;
% col_pos_end2_x10 = col_pos_end2;
% col_vel_end2_x10 = col_vel_end2;
% 
% 
% 
% col_data = col_data_z15;
% s1 = sin(col_data(:,2));
% s2 = sin(col_data(:,3));
% s3 = sin(col_data(:,4));
% 
% c1 = cos(col_data(:,2));
% c2 = cos(col_data(:,3));
% c3 = cos(col_data(:,4));
%      
% c23 = c2.*c3 - s2.*s3;
% s23 = s2.*c3 + c2.*s3;
%      
% % End effector position of master
% col_pos_end1 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];
% 
% col_vel_end1 = [];
% % col_f_end1 = [];
% % 
% for ii=1:1:size(col_pos_end1,1)
%     J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
%         Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
%         Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
%     
%     col_vel_end1(ii,:) = (J*[col_data(ii,8);col_data(ii,9);col_data(ii,10)])';
% %     col_f_end1(ii,:) = (pinv(J')*[col_tau1(ii,1);col_tau1(ii,2);col_tau1(ii,3)])';
% end
%  
% s1 = sin(col_data(:,5));
% s2 = sin(col_data(:,6));
% s3 = sin(col_data(:,7));
% 
% c1 = cos(col_data(:,5));
% c2 = cos(col_data(:,6));
% c3 = cos(col_data(:,7));
%      
% c23 = c2.*c3 - s2.*s3;
% s23 = s2.*c3 + c2.*s3;
%      
% % End effector position of slave
% col_pos_end2 = [Lknee*s23 + Lhip*s2, Labad*c1 + Lknee*s1.*c23 + Lhip*c2.*s1, Labad*s1 - Lknee*c1.*c23 - Lhip*c1.*c2];
% 
% col_vel_end2 = [];
% % col_f_end2 = [];
% 
% for ii=1:1:size(col_pos_end2,1)
%     J = [0, Lknee*c23(ii) + Lhip*c2(ii), Lknee*c23(ii);
%         Lknee*c1(ii)*c23(ii) + Lhip*c1(ii)*c2(ii) - Labad*s1(ii), -Lknee*s1(ii)*s23(ii) - Lhip*s1(ii)*s2(ii), -Lknee*s1(ii)*s23(ii);
%         Lknee*s1(ii)*c23(ii) + Lhip*c2(ii)*s1(ii) + Labad*c1(ii), Lknee*c1(ii)*s23(ii) + Lhip*c1(ii)*s2(ii), Lknee*c1(ii)*s23(ii)];
%     
%     col_vel_end2(ii,:) = (J*[col_data(ii,11);col_data(ii,12);col_data(ii,13)])';
% %     col_f_end2(ii,:) = (pinv(J')*[col_tau2(ii,1);col_tau2(ii,2);col_tau2(ii,3)])';
% end
% 
% col_pos_end1_z15 = col_pos_end1;
% col_vel_end1_z15 = col_vel_end1;
% col_pos_end2_z15 = col_pos_end2;
% col_vel_end2_z15 = col_vel_end2;


fprintf('done!\n');

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

p_std = std(free_pos_end1-free_pos_end2);
v_std = std(free_vel_end1-free_vel_end2);

p_std_j = std(free_data(:,2:4)-free_data(:,5:7));
v_std_j = std(free_data(:,8:10)-free_data(:,11:13));

pos_lim = 3*p_std %0.0035; %0:0.0001:0.0025;
vel_lim = 3*v_std %0.35; % 0:0.01:0.4;

% pos_lim_j = 3*p_std_j(2)
% vel_lim_j = 3*v_std_j(2)

col_bools_p = zeros(length(col_data(:,1)),length(pos_lim));
col_bools_v = zeros(length(col_data(:,1)),length(vel_lim));

% for ii=1:length(pos_lim)
%     plim = pos_lim(ii);        
%     col_bools_p(:,ii) = abs(col_pos_end1(:,1)-col_pos_end2(:,1))>plim;
%     
%     
%     fig = figure;
%     plot(col_data(:,1), col_pos_end2(:,1), col_data(:,1), col_bools_p(:,ii));
%     tl = sprintf('Plim: %.4f', pos_lim(ii));
%     title(tl)
%     
% %     filename = fullfile(pwd, 'Figures', sprintf('BoolClass_zvel_P%d',ii));
% %     print(fig, filename,'-dpng');
% %     close(fig);
%     
% end
% 
% for ii=1:length(vel_lim)
%     vlim = vel_lim(ii);
%     col_bools_v(:,ii) = abs(col_vel_end1(:,1)-col_vel_end2(:,1))>vlim;
%     
%     fig = figure;
%     plot(col_data(:,1), col_pos_end2(:,1), col_data(:,1), col_bools_v(:,ii));
%     tl = sprintf('Vlim: %.2f', vel_lim(ii));
%     title(tl)
%     
% %     filename = fullfile(pwd, 'Figures', sprintf('BoolClass_zvel_V%d',ii));
% %     print(fig, filename,'-dpng');
% %     close(fig);
% end

win = 5; % at 3 st devs, pos_lim = 0.0015 and vel_lim = 0.1310
col_bool_test = zeros(length(col_data(:,1)),3);

% col_bool_test_j = zeros(length(col_data(:,1)),1);

for ii=win:length(col_data(:,1))
    
    pwin = abs(col_pos_end1((ii-win+1):ii,:)-col_pos_end2((ii-win+1):ii,:));
    vwin = abs(col_vel_end1((ii-win+1):ii,:)-col_vel_end2((ii-win+1):ii,:));

%     pwin_j = abs(col_data((ii-win+1):ii,3)-col_data((ii-win+1):ii,6));
%     vwin_j = abs(col_data((ii-win+1):ii,9)-col_data((ii-win+1):ii,12));
    
    if (sum((pwin(:,1)>pos_lim(1))&(vwin(:,1)>vel_lim(1)))==win)
       col_bool_test(ii,1) = 1; 
    end  
    
    if (sum((pwin(:,2)>pos_lim(2))&(vwin(:,2)>vel_lim(2)))==win)
       col_bool_test(ii,2) = 1; 
    end 
    
    if (sum((pwin(:,3)>pos_lim(3))&(vwin(:,3)>vel_lim(3)))==win)
       col_bool_test(ii,3) = 1; 
    end 
    
%     if (sum((pwin_j>pos_lim_j)&(vwin_j>vel_lim_j))==win)
%        col_bool_test_j(ii) = 1; 
%     end  
    
end

figure;
plot(col_data(:,1), col_pos_end2(:,1), col_data(:,1), col_bool_test(:,1));
title('Pos and Vel cart conditions, z');

figure;
plot(col_data(:,1), col_pos_end2(:,2), col_data(:,1), col_bool_test(:,2));
title('Pos and Vel cart conditions, x');

figure;
plot(col_data(:,1), col_pos_end2(:,3), col_data(:,1), col_bool_test(:,3));
title('Pos and Vel cart conditions, y');

figure;
plot(col_data(:,1), col_detection(:,1), col_data(:,1), col_detection(:,2), col_data(:,1), col_detection(:,3), col_data(:,1), col_detection(:,4));
title('MCU reported col detection on z,x,y');

% figure;
% plot(col_data(:,1), col_pos_end2(:,1), col_data(:,1), col_bool_test_j);
% title('Pos and Vel joint conditions');


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

%% plotting test data with MCU collision detection information

% figure; plot(col_data_z10(1:10000,1),col_pos_end1_z10(1:10000,1),col_data_z10(1:10000,1),col_pos_end2_z10(1:10000,1),col_data_z10(1:10000,1),0.1*col_detection_z10(1:10000,1));
% figure; plot(col_data_z10(1:10000,1),col_vel_end1_z10(1:10000,1),col_data_z10(1:10000,1),col_detection_z10(1:10000,1)-1);

% % 
% figure; 
% yyaxis left; hold on;
% plot(col_data_x10(1:5000,1),col_pos_end1_x10(1:5000,2),'g','LineWidth',1.5);
% plot(col_data_x10(1:5000,1),col_pos_end2_x10(1:5000,2),'b','LineWidth',1.5);
% yyaxis right;
% plot(col_data_x10(1:5000,1),col_detection_x10(1:5000,1),'LineWidth',1.5);
% ylim([-0.2, 1.2]);


% figure; 
% yyaxis left; hold on;
% plot(col_data_z15(1:8000,1),col_pos_end1_z15(1:8000,1),'g','LineWidth',1.5);
% plot(col_data_z15(1:8000,1),col_pos_end2_z15(1:8000,1),'b','LineWidth',1.5);
% yyaxis right;
% plot(col_data_z15(1:8000,1),col_detection_z15(1:8000,1),'LineWidth',1.5);
% ylim([-0.2, 1.2]);
% figure; plot(col_data_z15(:,1),col_vel_end1_z15(:,1),col_data_z15(:,1),col_detection_z15(:,1)-1);

% with impact...0 to 7500
figure; subplot(2,2,1); yyaxis left; hold on;
plot(col_data_new(1:7500,1), col_pos_end1_new(1:7500,1),'g','LineWidth',1.5);
plot(col_data_new(1:7500,1), col_pos_end2_new(1:7500,1),'b','LineWidth',1.5);
hold off; 
ylim([-0.1, 0.15]);
legend('M','S','Col'); ylabel('Z-position, m'); xlabel('Time, s');
yyaxis right;
plot(col_data_new(1:7500,1), col_detection_new(1:7500,1));
ylim([-0.2, 1.2]); ylabel('Collision Flag');
subplot(2,2,3); hold on;
plot(col_data_new(1:7500,1), col_tau1_new(1:7500,2),'b','LineWidth',1.5);
plot(col_data_new(1:7500,1), col_tau1_new(1:7500,3),'r','LineWidth',1.5);
hold off; 
ylim([-5, 20]);
legend('Hip','Knee'); ylabel('Commanded Torque, Nm'); xlabel('Time, s');

% without impact...15000 to 20000
subplot(2,2,2); hold on;
plot(col_data_new(15000:20000,1)-14.999, col_pos_end1_new(15000:20000,1),'g','LineWidth',1.5);
plot(col_data_new(15000:20000,1)-14.999, col_pos_end2_new(15000:20000,1),'b','LineWidth',1.5);
hold off; 
xlim([0,5]); ylim([-0.1, 0.15]);
legend('M','S'); ylabel('Z-position, m'); xlabel('Time, s');

subplot(2,2,4); hold on;
plot(col_data_new(15000:20000,1)-14.999, col_tau1_new(15000:20000,2),'b','LineWidth',1.5);
plot(col_data_new(15000:20000,1)-14.999, col_tau1_new(15000:20000,3),'r','LineWidth',1.5);
hold off; 
xlim([0,5]); ylim([-5, 20]);
legend('Hip','Knee'); ylabel('Commanded Torque, Nm'); xlabel('Time, s');







