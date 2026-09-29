package org.firstinspires.ftc.teamcode.testrobot.opmodes.base;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.Scheduler;
import dev.nextftc.hardware.util.EventLoop;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.config.AutonomousConstants;
import org.firstinspires.ftc.teamcode.testrobot.utils.GamepadEffects;
import org.firstinspires.ftc.teamcode.testrobot.utils.PoseStorage;

/** Driver bindings shared by Main Driving and Recorder. */
public abstract class MainTeleOp extends RobotOpMode {
    private final EventLoop initEvents = new EventLoop();
    private Pose selectedStartPose;
    protected boolean crossPressed, circlePressed, importantEvent;
    private boolean started;

    protected MainTeleOp(Robot robot) { super(robot); }

    @Override protected final void initialize() {
        selectedStartPose = AutonomousConstants.leftStartPose();
        robot.initialize(hardwareMap, selectedStartPose);
        robot.drive.configureManual(gamepad1);
        CommandGamepad initPad = new CommandGamepad(gamepad1, initEvents);
        // Instant actions execute when scheduled. Use a separate event loop only in INIT.
        initPad.dpadLeft().onTrue(Commands.instant(() -> selectPose(AutonomousConstants.leftStartPose())));
        initPad.dpadRight().onTrue(Commands.instant(() -> selectPose(AutonomousConstants.rightStartPose())));
        initPad.dpadDown().onTrue(Commands.instant(() -> selectPose(PoseStorage.loadPose())));
        robot.afterHardwareUpdate(() -> {
            onRobotUpdated();
            crossPressed = circlePressed = importantEvent = false;
        });
        onRobotInit();
    }

    private void selectPose(Pose pose) {
        selectedStartPose = pose;
        robot.drive.setPose(pose);
    }

    @Override public final void disabledPeriodic() {
        initEvents.poll();
        Scheduler.execute();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Selected start", "(%.1f, %.1f, %.0f deg)",
                selectedStartPose.getX(), selectedStartPose.getY(), Math.toDegrees(selectedStartPose.getHeading()));
        telemetry.addLine("D-pad left: left start | right: right start | down: saved pose");
        Pose saved = PoseStorage.loadPose();
        telemetry.addData("Saved pose", "(%.2f, %.2f, %.0f deg)",
                saved.getX(), saved.getY(), Math.toDegrees(saved.getHeading()));
    }

    @Override protected final void onStart() {
        initEvents.clear();
        // NextFTC has already scheduled mechanism defaults; bind active controls now.
        bindDriverControls();
        GamepadEffects.startRgbEffect(gamepad1);
        robot.drive.startManual();
        started = true;
        onRobotStart();
    }

    private void bindDriverControls() {
        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.cross().onTrue(Commands.instant(() -> {
            crossPressed = importantEvent = true;
            if (!robot.drive.isFieldCentric() && !robot.drive.isAiming()) {
                robot.drive.driveToAndHold(AutonomousConstants.centerPose()).schedule();
            }
        }));
        driver.circle().onTrue(Commands.instant(() -> {
            circlePressed = importantEvent = true;
            robot.drive.manual().schedule();
        }));
        driver.triangle().onTrue(Commands.instant(() -> {
            importantEvent = true;
            if (robot.drive.isManual()) robot.drive.toggleFieldCentric();
        }));
        driver.square().onTrue(Commands.instant(() -> {
            importantEvent = true;
            if (robot.drive.isManual()) {
                if (robot.drive.isAiming()) robot.drive.stopAiming();
                else robot.drive.startAiming();
            }
        }));
        driver.rightStickButton().onTrue(Commands.instant(() -> {
            importantEvent = true;
            if (robot.drive.isAiming()) robot.drive.toggleAimingTarget();
        }));
        driver.leftStickButton().onTrue(Commands.instant(() -> robot.drive.setPose(
                robot.drive.getPose().getX() < 72
                        ? AutonomousConstants.leftStartPose() : AutonomousConstants.rightStartPose())));
        if (robot.intake != null) {
            driver.rightBumper().onTrue(robot.intake.toggle(false));
            driver.leftBumper().onTrue(robot.intake.toggle(true));
        }
    }

    @Override public final void periodic() {
        onRobotLoop();
        robot.drive.telemetry(telemetry);
        if (robot.intake != null) robot.intake.telemetry(telemetry);
        telemetry.addLine("X: center | Circle: manual | Triangle: field centric");
        telemetry.addLine("Square: aiming on/off | R3: blue/red target | L3: reset pose");
    }

    @Override protected final void beforeShutdown() {
        initEvents.clear();
        onRobotStopping(started);
    }

    protected void onRobotInit() {}
    protected void onRobotStart() {}
    protected void onRobotLoop() {}
    protected void onRobotUpdated() {}
    protected void onRobotStopping(boolean started) {}
}
