package artery.ros2;

import artery.application.Middleware;

simple RobotMiddleware extends Middleware
{
    parameters:
        @class(RobotMiddleware);
        @signal[IdentityChanged](type=long);

        string localEnvironmentModule = default("");
        string globalEnvironmentModule = default("");
        string mobilityModule;
}

