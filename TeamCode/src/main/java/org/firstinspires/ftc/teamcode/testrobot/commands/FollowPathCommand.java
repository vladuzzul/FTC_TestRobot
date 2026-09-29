package org.firstinspires.ftc.teamcode.testrobot.commands;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

/** A nonblocking path command with a monotonic timeout and explicit drive ownership. */
public final class FollowPathCommand extends CommandBuilder {
    private long startedAt;

    public FollowPathCommand(Object drive, Runnable begin, BooleanSupplier arrived,
                             Runnable stop, double timeoutSeconds) {
        this(drive, begin, arrived, stop, timeoutSeconds, System::nanoTime);
    }

    // Injectable clock keeps timeout and interruption tests independent of robot hardware.
    FollowPathCommand(Object drive, Runnable begin, BooleanSupplier arrived,
                      Runnable stop, double timeoutSeconds, LongSupplier nanoTime) {
        requiring(drive);
        setStart(() -> {
            startedAt = nanoTime.getAsLong();
            begin.run();
        });
        setDone(() -> arrived.getAsBoolean()
                || (nanoTime.getAsLong() - startedAt) / 1e9 >= timeoutSeconds);
        setEnd(reason -> {
            // Normal completion preserves Pedro's endpoint hold until the next path starts.
            if (reason != EndCondition.NATURALLY) stop.run();
        });
    }
}
