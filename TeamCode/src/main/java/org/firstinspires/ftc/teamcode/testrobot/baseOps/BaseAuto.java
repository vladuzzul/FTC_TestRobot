package org.firstinspires.ftc.teamcode.testrobot.baseOps;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.testrobot.Robot;

/** Shared lifecycle for PedroPathing autonomous OpModes. */
public abstract class BaseAuto extends OpMode {
    protected final Robot robot = Robot.getInstance();

    protected abstract Pose startingPose();

    @Override
    public void init() {
        robot.init(hardwareMap, startingPose());
        onAutoInit();
        telemetry.addData("Status", "Autonomous initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        robot.updateAllSystems();
        robot.drive.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        onAutoStart();
    }

    @Override
    public void loop() {
        onAutoLoop();
        robot.updateAllSystems();
        robot.drive.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
        onAutoStop();
    }

    protected void onAutoInit() {
    }

    protected void onAutoStart() {
    }

    protected void onAutoLoop() {
    }

    protected void onAutoStop() {
    }
}
