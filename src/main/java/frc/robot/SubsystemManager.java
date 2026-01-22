package frc.robot;

import frc.lib.Subsystem;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.loops.Looper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once.
 */
public class SubsystemManager implements ILooper {

    public static final double kRecheckDeviceIntervalSeconds = 10.0;

    private static SubsystemManager instance = null;

    private List<Subsystem> allSubsystems = new ArrayList<>();
    private final List<Loop> loops = new ArrayList<>();

    private SubsystemManager() {}

    public static SubsystemManager getInstance() {
        if (instance == null) {
            instance = new SubsystemManager();
        }
        return instance;
    }

    /**
     * Outputs telemetry from all subsystems to SmartDashboard.
     *
     * @param disabled whether the robot is disabled.
     */
    public void outputToSmartDashboard(boolean disabled) {
        allSubsystems.forEach(s -> s.outputTelemetry(disabled));
    }

    /**
     * Checks that all subsystems are healthy.
     *
     * @return true if all subsystems report as healthy.
     */
    public boolean checkSubsystems() {
        boolean allHealthy = true;
        for (Subsystem s : allSubsystems) {
            allHealthy &= s.checkSystem();
        }
        return allHealthy;
    }

    /**
     * Checks device configurations for all subsystems and rewrites any that have drifted.
     *
     * @return true if all device configurations are correct.
     */
    public boolean checkDeviceConfigurations() {
        boolean allConfigsOk = true;
        for (Subsystem s : allSubsystems) {
            boolean devicesOk = s.checkDeviceConfiguration();
            if (!devicesOk) {
                s.rewriteDeviceConfiguration();
                DriverStation.reportWarning("Rewriting configuration after detecting configuration drift for subsystem: " + s.toString(), false);
            }
            allConfigsOk &= devicesOk;
        }
        return allConfigsOk;
    }

    /**
     * Stops all subsystems.
     */
    public void stop() {
        allSubsystems.forEach(Subsystem::stop);
    }

    /**
     * Returns an unmodifiable list of all subsystems.
     *
     * @return the list of subsystems.
     */
    public List<Subsystem> getSubsystems() {
        return Collections.unmodifiableList(allSubsystems);
    }

    /**
     * Registers the given subsystems.
     *
     * @param subsystems the subsystems to register.
     */
    public void setSubsystems(Subsystem... subsystems) {
        allSubsystems = new ArrayList<>(Arrays.asList(subsystems));
    }

    /**
     * Adds a single subsystem to the manager.
     *
     * @param subsystem the subsystem to add.
     */
    public void addSubsystem(Subsystem subsystem) {
        allSubsystems.add(subsystem);
    }

    private class EnabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
            loops.forEach(loop -> loop.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            allSubsystems.forEach(Subsystem::readPeriodicInputs);
            loops.forEach(loop -> loop.onLoop(timestamp));
            allSubsystems.forEach(Subsystem::writePeriodicOutputs);
        }

        @Override
        public void onStop(double timestamp) {
            loops.forEach(loop -> loop.onStop(timestamp));
        }
    }

    private class DisabledLoop implements Loop {
        private double lastCheckTimeSeconds = 0;

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            allSubsystems.forEach(Subsystem::readPeriodicInputs);
            // Periodically check that device configurations are OK.
            if ((Timer.getFPGATimestamp() - lastCheckTimeSeconds) > kRecheckDeviceIntervalSeconds) {
                lastCheckTimeSeconds = Timer.getFPGATimestamp();
                checkDeviceConfigurations();
            }
        }

        @Override
        public void onStop(double timestamp) {}
    }

    @Override
    public void register(Loop loop) {
        loops.add(loop);
    }

    /**
     * Registers loops to be run when the robot is enabled.
     *
     * @param enabledLooper the looper that runs when enabled.
     */
    public void registerEnabledLoops(Looper enabledLooper) {
        allSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    /**
     * Registers loops to be run when the robot is disabled.
     *
     * @param disabledLooper the looper that runs when disabled.
     */
    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }
}
