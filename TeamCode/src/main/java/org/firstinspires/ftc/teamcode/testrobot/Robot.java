package org.firstinspires.ftc.teamcode.testrobot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.testrobot.utils.misc.LpsCounter;
import org.firstinspires.ftc.teamcode.testrobot.utils.components.DriveController;
import org.firstinspires.ftc.teamcode.testrobot.utils.components.IntakeController;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;

public final class Robot extends TestRobot {
    private static Robot instance;

    public final DriveController drive;

    public final LpsCounter lpsCounter;

    private static final double ROBOT_RADIUS = 9.0;

    private final FieldManager panelsField =
            PanelsField.INSTANCE.getField();

    private final Style robotStyle =
            new Style("", "#a10342", 0.75);

    /// Uncomment to enable the intake
//    public final IntakeController intake;

    private Robot() {
        drive = new DriveController(this);
        lpsCounter = new LpsCounter();
        /// Uncomment to enable the intake
//        intake = new IntakeController();
    }

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap, Pose startingPose) {
        initFollower(hardwareMap, startingPose);
        drive.init();
        panelsField.setOffsets(
                PanelsField.INSTANCE.getPresets().getPEDRO_PATHING()
        );
        /// Uncomment to enable the intake
//        intake.init(hardwareMap);
    }

    public void updateAllSystems() {
        follower.update();
        drawRobotOnPanels();
    }

    private void drawRobotOnPanels() {
        Pose pose = follower.getPose();

        if (pose == null
                || !Double.isFinite(pose.getX())
                || !Double.isFinite(pose.getY())
                || !Double.isFinite(pose.getHeading())) {
            return;
        }

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        panelsField.setStyle(robotStyle);

        // Robot body
        panelsField.moveCursor(x, y);
        panelsField.circle(ROBOT_RADIUS);

        // Heading indicator
        double headingX = Math.cos(heading) * ROBOT_RADIUS;
        double headingY = Math.sin(heading) * ROBOT_RADIUS;

        panelsField.moveCursor(
                x + headingX / 2.0,
                y + headingY / 2.0
        );
        panelsField.line(
                x + headingX,
                y + headingY
        );

        // Send this frame to the Field widget
        panelsField.update();
    }
}
