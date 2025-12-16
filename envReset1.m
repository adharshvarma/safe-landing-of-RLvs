function [InitialObservation, LoggedSignals] = envReset1()

    x0  = 0;
    z0  = -2000;
    vx0 = 0;
    vz0 = 0;

    InitialObservation = [x0; z0; vx0; vz0];
    LoggedSignals.State = InitialObservation;
end
