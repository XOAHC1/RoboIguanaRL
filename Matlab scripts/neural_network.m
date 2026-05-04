clc; clear; close all;

%% =========================
%  User setting: choose one target
%  =========================
targetName = "deviation";   % choose: "speed", "CoT", or "deviation"

%% =========================
%  Load CSV file
%  =========================
filename = 'experiment_summary.csv';
T = readtable(filename, 'TextType', 'string');

disp('First few rows of the dataset:')
disp(head(T))

%% =========================
%  Ignore phase column
%  =========================
if any(strcmpi(T.Properties.VariableNames, 'phase'))
    T.phase = [];
end

%% =========================
%  Standardize spinemode
%  =========================
T.spinemode = lower(strtrim(T.spinemode));
T.spinemode = categorical(T.spinemode, {'rigid','flexible','active'});

if any(ismissing(T.spinemode))
    error('Some rows in spinemode are invalid. Allowed values: rigid, flexible, active.');
end

%% =========================
%  One-hot encode spinemode
%  =========================
modeOH = onehotencode(T.spinemode, 2);   % [rigid flexible active]

%% =========================
%  Build input X
%  Inputs: [frequency, rigid, flexible, active, spineAmp, spineStiff]
%  =========================
X = [T.frequency, modeOH, T.spineAmp, T.spineStiff];

%% =========================
%  Select one output Y
%  =========================
switch targetName
    case "speed"
        Y = T.speed;
    case "CoT"
        Y = T.CoT;
    case "deviation"
        Y = T.deviation;
    otherwise
        error('targetName must be "speed", "CoT", or "deviation".');
end

%% =========================
%  Remove rows with missing values
%  =========================
validRows = all(~isnan(X),2) & ~isnan(Y);
X = X(validRows,:);
Y = Y(validRows,:);

fprintf('Target variable: %s\n', targetName);
fprintf('Total valid samples before outlier removal: %d\n', size(X,1));

%% =========================
%  Remove outliers in target Y using IQR rule
%  =========================
Q1 = prctile(Y, 25);
Q3 = prctile(Y, 75);
IQRy = Q3 - Q1;

lowerBound = Q1 - 1 * IQRy;
upperBound = Q3 + 1 * IQRy;

keepRows = (Y >= lowerBound) & (Y <= upperBound);

nRemoved = sum(~keepRows);

X = X(keepRows,:);
Y = Y(keepRows,:);

fprintf('Outlier bounds for %s: [%.6f, %.6f]\n', targetName, lowerBound, upperBound);
fprintf('Removed outliers: %d\n', nRemoved);
fprintf('Total valid samples after outlier removal: %d\n', size(X,1));

%% =========================
%  Optional: visualize target before/after outlier removal
%  =========================
figure;
subplot(1,2,1)
boxplot(Y)
title(['Boxplot after outlier removal: ', char(targetName)])

subplot(1,2,2)
histogram(Y)
title(['Histogram after outlier removal: ', char(targetName)])

%% =========================
%  Train / validation / test split
%  =========================
rng(1);

N = size(X,1);
idx = randperm(N);

nTrain = round(0.70*N);
nVal   = round(0.15*N);
nTest  = N - nTrain - nVal;

idxTrain = idx(1:nTrain);
idxVal   = idx(nTrain+1:nTrain+nVal);
idxTest  = idx(nTrain+nVal+1:end);

XTrain = X(idxTrain,:);
YTrain = Y(idxTrain,:);

XVal = X(idxVal,:);
YVal = Y(idxVal,:);

XTest = X(idxTest,:);
YTest = Y(idxTest,:);

%% =========================
%  Normalize only continuous inputs
%  =========================
contCols = [1 5 6];   % frequency, spineAmp, spineStiff

muX = mean(XTrain(:,contCols), 1);
sigX = std(XTrain(:,contCols), 0, 1);
sigX(sigX == 0) = 1;

XTrainN = XTrain;
XValN   = XVal;
XTestN  = XTest;

XTrainN(:,contCols) = (XTrain(:,contCols) - muX) ./ sigX;
XValN(:,contCols)   = (XVal(:,contCols)   - muX) ./ sigX;
XTestN(:,contCols)  = (XTest(:,contCols)  - muX) ./ sigX;

%% =========================
%  Normalize output
%  =========================
muY = mean(YTrain,1);
sigY = std(YTrain,0,1);
if sigY == 0
    sigY = 1;
end

YTrainN = (YTrain - muY) ./ sigY;
YValN   = (YVal   - muY) ./ sigY;
YTestN  = (YTest  - muY) ./ sigY;

%% =========================
%  Define neural network
%  =========================
layers = [
    featureInputLayer(6, Normalization="none", Name="input")
    fullyConnectedLayer(16, Name="fc1")
    reluLayer(Name="relu1")
    fullyConnectedLayer(8, Name="fc2")
    reluLayer(Name="relu2")
    fullyConnectedLayer(1, Name="output")
];

%% =========================
%  Training options
%  =========================
options = trainingOptions("adam", ...
    MaxEpochs=300, ...
    MiniBatchSize=4, ...
    InitialLearnRate=3e-4, ...
    L2Regularization=1e-3, ...
    Shuffle="every-epoch", ...
    ValidationData={XValN, YValN}, ...
    ValidationFrequency=10, ...
    Verbose=true, ...
    Plots="training-progress");

%% =========================
%  Train network
%  =========================
net = trainnet(XTrainN, YTrainN, layers, "mse", options);

%% =========================
%  Predict on test set
%  =========================
YPredN = predict(net, XTestN);

% De-normalize predictions
YPred = YPredN .* sigY + muY;

%% =========================
%  Metrics
%  =========================
rmse = sqrt(mean((YPred - YTest).^2));
mae  = mean(abs(YPred - YTest));

SSres = sum((YTest - YPred).^2);
SStot = sum((YTest - mean(YTest)).^2);
R2 = 1 - SSres / SStot;

fprintf('\nTest results for %s:\n', targetName);
fprintf('RMSE = %.6f\n', rmse);
fprintf('MAE  = %.6f\n', mae);
fprintf('R^2  = %.6f\n', R2);

%% =========================
%  Plot predicted vs actual
%  =========================
figure;
scatter(YTest, YPred, 50, 'filled');
hold on;
mn = min([YTest; YPred]);
mx = max([YTest; YPred]);
plot([mn mx], [mn mx], 'k--', 'LineWidth', 1.5);
xlabel("Actual " + targetName);
ylabel("Predicted " + targetName);
title("Predicted vs Actual: " + targetName);
grid on;

%% =========================
%  Optional residual plot
%  =========================
% residuals = YPred - YTest;
%
% figure;
% scatter(YTest, residuals, 50, 'filled');
% hold on;
% yline(0, 'k--', 'LineWidth', 1.5);
% xlabel("Actual " + targetName);
% ylabel("Residual");
% title("Residual Plot: " + targetName);
% grid on;

