package artery.ros2;

import artery.networking.IPositionProvider;

module RobotPositionProvider like IPositionProvider
{
    parameters:
        @class(RobotPositionProvider);
        @signal[PositionFix](PositionFixObject);
        @signal[robotPositionChanged](type=ros2::RobotObject);
        @signal[simulationPositionChanged](type=ros2::RobotObject);
        string runtimeModule;
        string mobilityModule;
}
