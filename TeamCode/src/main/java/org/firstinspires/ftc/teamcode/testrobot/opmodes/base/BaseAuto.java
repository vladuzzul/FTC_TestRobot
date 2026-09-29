package org.firstinspires.ftc.teamcode.testrobot.opmodes.base;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.testrobot.Robot;

/** Autonomous OpModes describe a command instead of advancing a switch-based state machine. */
public abstract class BaseAuto extends RobotOpMode {
    private Command autonomous;

    protected BaseAuto(Robot robot) { super(robot); }
    protected abstract Pose startingPose();
    protected abstract Command autonomousCommand();

    @Override protected final void initialize() {
        robot.initialize(hardwareMap, startingPose());
        autonomous = autonomousCommand();
    }

    @Override protected final void onStart() { autonomous.schedule(); }

    @Override public void disabledPeriodic() {
        telemetry.addData("Status", "Autonomous initialized");
        robot.drive.telemetry(telemetry);
    }

    @Override public void periodic() {
        telemetry.addData("Autonomous", autonomous.isScheduled() ? "Running" : "Complete / holding");
        robot.drive.telemetry(telemetry);
    }

    @Override protected void beforeShutdown() {
        if (autonomous != null) autonomous.cancel();
    }
}
