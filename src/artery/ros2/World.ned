package artery.ros2;

import inet.common.geometry.common.IGeographicCoordinateSystem;
import inet.environment.contract.IPhysicalEnvironment;
import inet.physicallayer.contract.packetlevel.IRadioMedium;
import ros2.Connector;

network World
{
    parameters:
        bool withPhysicalEnvironment = default(false);
        int numRoadSideUnits = default(0);

    submodules:
        ros2: Connector {
            parameters:
                @display("p=20,20");
                robotController.creationPolicy.moduleType = default("artery.ros2.Robot");
                robotController.creationPolicy.robotSinkPath = default(".mobility");
        }

        radioMedium: <default("Ieee80211ScalarRadioMedium")> like IRadioMedium {
            parameters:
                @display("p=60,20");
                mediumLimitCache.carrierFrequency = 5.9GHz;
        }

        physicalEnvironment: <default("PhysicalEnvironment")> like IPhysicalEnvironment if withPhysicalEnvironment {
            parameters:
                @display("p=100,20");
        }

        coordinateSystem: <default("SimpleGeographicCoordinateSystem")> like IGeographicCoordinateSystem {
            parameters:
                @display("p=140,20");
        }

        rsu[numRoadSideUnits]: RSU {
            parameters:
                mobility.initFromDisplayString = false;
        }
}
