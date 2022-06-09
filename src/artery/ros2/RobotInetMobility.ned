package artery.ros2;

import inet.mobility.contract.IMobility;

simple RobotInetMobility like IMobility
{
    parameters:
        @signal[mobilityStateChanged];
        @signal[robotPositionChanged](type=ros2::RobotObject);
        @signal[localizationPositionChanged](type=ros2::RobotObject);
        string visualRepresentation = default("");
        string coordinateSystemModule = default("coordinateSystem");
}
