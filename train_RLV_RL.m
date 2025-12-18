clear; clc; close all;

%% Observation spec
obsInfo = rlNumericSpec([4 1], 'Name','states');

%% Action spec (continuous theta in radians)
actInfo = rlNumericSpec([1 1], ...
    'LowerLimit', deg2rad(-20), ...
    'UpperLimit', deg2rad(20), ...
    'Name','theta');

%% Environment
env = rlFunctionEnv(obsInfo, actInfo, ...
    "envStep", "envReset");

%% ===== Actor Network (Gaussian Continuous) =====
actorNet = layerGraph();

% Input
actorNet = addLayers(actorNet, ...
    featureInputLayer(4,'Normalization','none','Name','state'));

% Common layers
actorNet = addLayers(actorNet, [
    fullyConnectedLayer(64,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(64,'Name','fc2')
    reluLayer('Name','relu2')
]);

% Mean output
actorNet = addLayers(actorNet, ...
    fullyConnectedLayer(1,'Name','mean'));

% Std output (must be positive)
actorNet = addLayers(actorNet, [
    fullyConnectedLayer(1,'Name','std')
    softplusLayer('Name','std_plus')
]);

% Connections
actorNet = connectLayers(actorNet,'state','fc1');
actorNet = connectLayers(actorNet,'relu2','mean');
actorNet = connectLayers(actorNet,'relu2','std');

%% Actor
actor = rlContinuousGaussianActor( ...
    actorNet, obsInfo, actInfo, ...
    'ActionMeanOutputNames','mean', ...
    'ActionStandardDeviationOutputNames','std_plus');

%% ===== Critic Network =====
criticNet = [
    featureInputLayer(4,'Normalization','none')
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(1)
];

critic = rlValueFunction(criticNet, obsInfo);

%% PPO Agent options
agentOpts = rlPPOAgentOptions( ...
    'ExperienceHorizon',512, ...
    'ClipFactor',0.2, ...
    'EntropyLossWeight',0.005);

agent = rlPPOAgent(actor, critic, agentOpts);

%% Training options
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',2000, ...
    'MaxStepsPerEpisode',1000, ...
    'Verbose',true, ...
    'Plots','training-progress');

%% Train
train(agent, env, trainOpts);
