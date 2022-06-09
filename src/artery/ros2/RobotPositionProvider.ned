package artery.ros2;

import artery.networking.IPositionProvider;

module RobotPositionProvider like IPositionProvider
{
    parameters:
        @class(RobotPositionProvider);
        @signal[PositionFix](PositionFixObject);
        string runtimeModule;
        string mobilityModule;
}
