package ros2;

module BasicRobotLifecycleController like RobotLifecycleController
{
    parameters:
        @class(BasicRobotLifecycleController);
        volatile double insertionDelay = default(uniform(0s, 0.1s)) @unit(s);

    submodules:
        creationPolicy: <default("UniformRobotCreationPolicy")> like RobotCreationPolicy {
        }
}
