package org.firstinspires.ftc.teamcode.testrobot.opmodes.base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.utils.PoseStorage;

/** Shared resource lifecycle; NextFTC owns the scheduler and active mechanism updates. */
public abstract class RobotOpMode extends NextOpMode {
    protected final Robot robot;
    private boolean closed;

    protected RobotOpMode(Robot robot) { this(robot, new SessionHook()); }

    private RobotOpMode(Robot robot, SessionHook session) {
        super(robot, session);
        this.robot = robot;
        session.owner = this;
    }

    protected abstract void initialize();
    protected void onStart() {}
    protected void beforeShutdown() {}

    @Override public final void start() {
        // In 0.2.1 the bound lifecycle reaches start() even if STOP was pressed in INIT.
        LinearOpMode active = (LinearOpMode) OpModeManagerImpl
                .getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).getActiveOpMode();
        if (!active.isStopRequested()) onStart();
    }

    @Override public final void end() { shutdown(); }

    private void shutdown() {
        if (closed) return;
        closed = true;
        // Stop before file I/O; release hardware even if a recorder fails to close.
        try {
            robot.stop();
        } finally {
            try {
                beforeShutdown();
            } finally {
                try {
                    if (robot.drive.isInitialized()) PoseStorage.savePose(robot.drive.getPose());
                } finally {
                    robot.release();
                }
            }
        }
    }

    private static final class SessionHook implements OpModeHook {
        private RobotOpMode owner;
        @Override public void afterConstruction() { owner.initialize(); }
        @Override public void beforeDisabled() {
            // NextFTC 0.2.1 only ticks mechanisms during PLAY; refresh localization in INIT.
            owner.robot.drive.periodic();
        }
        @Override public void afterEnd() {
            // The bound OpMode calls this from finally, including when initialization throws.
            owner.shutdown();
        }
    }
}
