clear; clc; close all;

%% Observation and Action specs
obsInfo = rlNumericSpec([4 1], 'Name','states');

actInfo = rlFiniteSetSpec([1 2 3], 'Name','theta_index');

%% Environment
env = rlFunctionEnv(obsInfo, actInfo, ...
    "envStep1", "envReset1");

%% Actor (DISCRETE / categorical)
actorNet = [
    featureInputLayer(4,'Normalization','none')
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(3)      % 3 actions
    softmaxLayer                % probabilities
];

actor = rlDiscreteCategoricalActor(actorNet, obsInfo, actInfo);

%% Critic
criticNet = [
    featureInputLayer(4,'Normalization','none')
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(1)
];

critic = rlValueFunction(criticNet, obsInfo);

%% PPO Agent
agentOpts = rlPPOAgentOptions( ...
    'ExperienceHorizon',256, ...
    'ClipFactor',0.2, ...
    'EntropyLossWeight',0.01);

agent = rlPPOAgent(actor, critic, agentOpts);

%% Training options
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',2000, ...
    'MaxStepsPerEpisode',1000, ...
    'Verbose',true, ...
    'Plots','training-progress');

%% Train
train(agent, env, trainOpts);
