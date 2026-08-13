package org.firstinspires.ftc.teamcode.testrobot.baseOps;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.autonomous.AutonomousConstants;
import org.firstinspires.ftc.teamcode.testrobot.utils.Controls;
import org.firstinspires.ftc.teamcode.testrobot.utils.PoseStorage;

/** Shared lifecycle and driver controls for TestRobot TeleOp programs. */
public abstract class MainTeleOp extends OpMode {
    protected final Robot robot = Robot.getInstance();
    protected Controls controls;

    private Pose selectedStartPose;

    @Override
    public void init() {
        controls = new Controls(gamepad1);
        selectedStartPose = AutonomousConstants.leftStartPose();
        robot.init(hardwareMap, selectedStartPose);
        onRobotInit();
        initTelemetry();
    }

    @Override
    public void init_loop() {
        controls.update();

        if (controls.dpadLeft.justPressed()) {
            selectedStartPose = AutonomousConstants.leftStartPose();
            robot.setPose(selectedStartPose);
        } else if (controls.dpadRight.justPressed()) {
            selectedStartPose = AutonomousConstants.rightStartPose();
            robot.setPose(selectedStartPose);
        }
        else if (controls.dpadDown.justPressed()) {
            selectedStartPose = PoseStorage.loadPose();
            robot.setPose(selectedStartPose);
        }

        robot.updateAllSystems();
        initTelemetry();
    }

    @Override
    public void start() {
        robot.drive.startManual();
        onRobotStart();
    }

    @Override
    public void loop() {
        controls.update();

        if (controls.a.justPressed()) {
            robot.drive.driveTo(AutonomousConstants.centerPose());
        }

        if (controls.b.justPressed()) {
            robot.drive.startManual();
        }

        robot.drive.runManual(controls);
        onRobotLoop();
        robot.updateAllSystems();
        onRobotUpdated();

        robot.drive.telemetry(telemetry);
        telemetry.addLine("A -> Go to center, B -> Manual");
        telemetry.update();
    }

    @Override
    public void stop() {
        onRobotStopping();
        PoseStorage.savePose(robot.follower.getPose());
        robot.stop();
        onRobotStop();
    }

    protected void onRobotInit() {
    }

    protected void onRobotStart() {
    }

    protected void onRobotLoop() {
    }

    /** Called once per loop after every robot system has been updated. */
    protected void onRobotUpdated() {
    }

    /** Called while the final live robot state is still available. */
    protected void onRobotStopping() {
    }

    protected void onRobotStop() {
    }

    private void initTelemetry() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Selected start", "(%.1f, %.1f, %.0f deg)",
                selectedStartPose.getX(), selectedStartPose.getY(),
                Math.toDegrees(selectedStartPose.getHeading()));
        telemetry.addLine("Select start left-right arrow");
        telemetry.update();
    }
}
