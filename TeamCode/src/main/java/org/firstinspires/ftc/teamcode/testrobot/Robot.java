package org.firstinspires.ftc.teamcode.testrobot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import org.firstinspires.ftc.teamcode.testrobot.mechanisms.Drive;
import org.firstinspires.ftc.teamcode.testrobot.mechanisms.Intake;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

/** The sole discoverable NextRobot. Construction must not access live hardware. */
public final class Robot implements NextRobot {
    // The intake was disabled in the original robot. Enable only when fitted/configured.
    public static final boolean INTAKE_ENABLED = false;
    public final Drive drive = new Drive();
    public final Intake intake = INTAKE_ENABLED ? new Intake() : null;
    private Runnable afterHardwareUpdate = () -> {};
    private final Set<Mechanism> mechanisms;

    public Robot() {
        LinkedHashSet<Mechanism> ordered = new LinkedHashSet<>();
        ordered.add(drive);
        if (intake != null) ordered.add(intake);
        // Runs after Pedro's update, before the scheduler prepares the next cycle's inputs.
        ordered.add(new Mechanism() {
            @Override public void periodic() { afterHardwareUpdate.run(); }
        });
        mechanisms = Collections.unmodifiableSet(ordered);
    }

    @Override public Set<Mechanism> getMechanisms() { return mechanisms; }

    public void initialize(HardwareMap hardwareMap, Pose startingPose) {
        afterHardwareUpdate = () -> {};
        drive.initialize(hardwareMap, startingPose);
        if (intake != null) intake.initialize(hardwareMap);
    }

    public void afterHardwareUpdate(Runnable callback) { afterHardwareUpdate = callback; }

    public void stop() {
        try { drive.stop(); }
        finally { if (intake != null) intake.stopMotor(); }
    }

    public void release() {
        afterHardwareUpdate = () -> {};
        try { drive.release(); }
        finally { if (intake != null) intake.release(); }
    }
}
