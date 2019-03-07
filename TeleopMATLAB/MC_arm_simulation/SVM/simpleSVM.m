% build and train simple SVM to get process down, then move to OC-SVM for
% collision detection between teleoperation

% may be some interesting considerations in coll.det. SVM due to forces
% from teleop coupling

%% generate data
data1 = [ones(1000,1), 10+(3*randn(1000,1)), 10+(3*randn(1000,1)), 10+(3*randn(1000,1))];
data2 = [-1*ones(1000,1), -10+(3*randn(1000,1)), -5+(4*randn(1000,1)), -10+(3*randn(1000,1))];

data = [data1; data2];

%% separate to training and testing sets

% don't really need this for now, can just generate more data

%% training

% max_feature_value = max(max([data1(:,2:end); data2(:,2:end)]));
% min_feature_value = min(min([data1(:,2:end); data2(:,2:end)]));
% learning_rates = max_feature_value*[0.1, 0.01, 0.001];
% 
% % want [x,y].[w1,w2] + b >= +1 for data1 
% % and [x,y].[w1,w2] + b <= -1 for data2

%%   train using matlab toolbox
SVMModel = fitcsvm(data(:,2:4),data(:,1));

%% investigate

pred_data = [10*randn(100,1), 3+10*randn(100,1), 10*randn(100,1)];

[label,score] = predict(SVMModel,pred_data);

figure; 
scatter3(data1(:,2),data1(:,3),data1(:,4))

hold on
scatter3(data2(:,2),data2(:,3),data2(:,4))

for ii=1:1:100
    if label(ii)==1
        plot3(pred_data(ii,1),pred_data(ii,2),pred_data(ii,3),'b*');
    else
        plot3(pred_data(ii,1),pred_data(ii,2),pred_data(ii,3),'r*');
    end
end

w = SVMModel.Beta;
b = SVMModel.Bias;

[xp, yp] = meshgrid(-25:25:25);
zp = (w(1)*xp + w(2)*yp + b)/(-w(3));

surf(xp,yp,zp,'FaceAlpha',0.25,'EdgeColor','k','FaceColor','y');

hold off













