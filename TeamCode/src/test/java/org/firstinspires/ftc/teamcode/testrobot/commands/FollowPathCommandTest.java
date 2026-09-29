package org.firstinspires.ftc.teamcode.testrobot.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;

public class FollowPathCommandTest {
    private final Object drive = new Object();
    private final AtomicLong clock = new AtomicLong();
    private final AtomicBoolean arrived = new AtomicBoolean();
    private final AtomicInteger starts = new AtomicInteger();
    private final AtomicInteger stops = new AtomicInteger();

    @Before public void resetBefore() { Scheduler.reset(); }
    @After public void resetAfter() { Scheduler.reset(); }

    private Command path() {
        return new FollowPathCommand(drive, starts::incrementAndGet, arrived::get,
                stops::incrementAndGet, 5, clock::get);
    }

    @Test public void pathOwnsDriveUntilArrivalAndPreservesEndpointHold() {
        Command path = path();
        path.schedule();
        Scheduler.execute();
        assertTrue(path.isScheduled());
        assertTrue(path.requirements().contains(drive));
        arrived.set(true);
        Scheduler.execute();
        assertFalse(path.isScheduled());
        assertEquals(1, starts.get());
        assertEquals(0, stops.get());
    }

    @Test public void timeoutAdvancesWithoutSleeping() {
        Command path = path();
        path.schedule();
        clock.set(4_999_999_999L);
        Scheduler.execute();
        assertTrue(path.isScheduled());
        clock.incrementAndGet();
        Scheduler.execute();
        assertFalse(path.isScheduled());
        assertEquals(0, stops.get());
    }

    @Test public void manualOverrideStopsPathBeforeStartingManual() {
        Command path = path();
        path.schedule();
        AtomicBoolean manualStarted = new AtomicBoolean();
        Commands.instant(() -> {
            assertEquals(1, stops.get());
            manualStarted.set(true);
        }).requiring(drive).schedule();
        assertFalse(path.isScheduled());
        assertTrue(manualStarted.get());
    }

    @Test public void cancelledPathLetsSuspendedDefaultDriveResume() {
        AtomicInteger manualTicks = new AtomicInteger();
        Command manual = Commands.infinite(manualTicks::incrementAndGet).requiring(drive)
                .setPriority(Integer.MIN_VALUE).setInterruptedBehavior(InterruptedBehavior.SUSPEND);
        manual.schedule();
        Scheduler.execute();
        assertEquals(1, manualTicks.get());
        Command path = path();
        path.schedule();
        Scheduler.execute();
        assertEquals(1, manualTicks.get());
        path.cancel();
        Scheduler.execute(); // Ivy returns the suspended command to the runnable queue.
        Scheduler.execute();
        assertEquals(2, manualTicks.get());
        assertEquals(1, stops.get());
    }

    @Test public void sequenceKeepsOwnershipAcrossConsecutivePaths() {
        Command sequence = Groups.sequential(path(), path());
        sequence.schedule();
        assertEquals(1, starts.get());
        arrived.set(true);
        Scheduler.execute();
        assertEquals(2, starts.get());
        assertTrue(sequence.isScheduled());
        Scheduler.execute();
        assertFalse(sequence.isScheduled());
        assertEquals(0, stops.get());
    }

    @Test public void deadlineInterruptsCycleThenStartsParking() {
        AtomicBoolean deadline = new AtomicBoolean();
        AtomicBoolean parked = new AtomicBoolean();
        Command sequence = Groups.sequential(path().until(deadline::get),
                Commands.instant(() -> {
                    assertEquals(1, stops.get());
                    parked.set(true);
                }).requiring(drive));
        sequence.schedule();
        Scheduler.execute();
        assertFalse(parked.get());
        deadline.set(true);
        Scheduler.execute();
        Scheduler.execute();
        assertTrue(parked.get());
    }

    @Test public void reschedulingRestartsTimeout() {
        Command path = path();
        path.schedule();
        clock.set(5_000_000_000L);
        Scheduler.execute();
        path.schedule();
        Scheduler.execute();
        assertTrue(path.isScheduled());
        assertEquals(2, starts.get());
    }
}
