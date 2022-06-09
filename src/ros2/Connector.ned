package ros2;

module Connector
{
    parameters:
        @display("i=block/network2;is=s");

    submodules:
        core: Core {
        }

        robotController: <default("BasicRobotLifecycleController")> like RobotLifecycleController {
        }
}
