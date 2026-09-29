package org.firstinspires.ftc.teamcode.testrobot.utils;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;

/** Panels field rendering, independent of the command scheduler. */
public final class FieldDrawing {
    private static final double ROBOT_RADIUS = 9.0;
    private final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private final Style robotStyle = new Style("", "#a10342", 0.75);

    public void initialize() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public void draw(Pose pose) {

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
