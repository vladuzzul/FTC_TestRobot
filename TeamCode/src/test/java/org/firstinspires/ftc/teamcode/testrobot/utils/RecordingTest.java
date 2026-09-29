package org.firstinspires.ftc.teamcode.testrobot.utils;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import static org.junit.Assert.*;

public class RecordingTest {
    @Rule public TemporaryFolder temp = new TemporaryFolder();
    private static final String ROW =
            "0,0.4,0.2,0.8,0.6,9,9,1.5708,13,0.7,-0.2,0.1,0,1,0,2,3,0.4,1,1,1";

    private File file(String text) throws IOException {
        File file = temp.newFile();
        try (FileWriter writer = new FileWriter(file)) { writer.write(text); }
        return file;
    }

    private String rowWith(int index, String value) {
        String[] values = ROW.split(",");
        values[index] = value;
        return String.join(",", values);
    }

    @Test public void readsExisting21ColumnRecordingAndManualInputs() throws IOException {
        List<Recording.RobotFrame> frames = Recording.load(file(RecordingFormat.HEADER + "\n" + ROW));
        assertEquals(1, frames.size());
        Recording.RobotFrame first = frames.get(0);
        assertEquals(9, first.x, 0);
        assertEquals(1.5708, first.heading, 0);
        assertTrue(first.aPressed);
        assertFalse(first.bPressed);
        assertTrue(first.fieldCentric && first.aimingAtTarget && first.aimingAtBlue);
        assertEquals(0.7, first.feedforward().forward, 1e-9);
        assertEquals(-0.2, first.feedforward().strafe, 1e-9);
        assertEquals(0.1, first.feedforward().turn, 1e-9);
    }

    @Test public void reconstructsPathAndHoldInputsFromRecordedMotorPowers() throws IOException {
        for (int mode : new int[]{1, 2}) {
            Recording.DriveCommand input = Recording.load(file(RecordingFormat.HEADER + "\n"
                    + rowWith(12, String.valueOf(mode)))).get(0).feedforward();
            assertEquals(0.5, input.forward, 1e-9);
            assertEquals(0, input.strafe, 1e-9);
            assertEquals(0.1, input.turn, 1e-9);
        }
    }

    @Test public void acceptsBooleanWordsAndDuplicateTimestamps() throws IOException {
        List<Recording.RobotFrame> frames = Recording.load(file(RecordingFormat.HEADER + "\n"
                + rowWith(20, "true") + "\n\n" + rowWith(20, "false")));
        assertEquals(2, frames.size());
        assertTrue(frames.get(0).aimingAtBlue);
        assertFalse(frames.get(1).aimingAtBlue);
    }

    @Test public void rejectsNonFiniteDriveDataBeforeScheduling() throws IOException {
        for (int column : new int[]{0, 1, 5, 8, 9, 15, 17}) {
            File csv = file(RecordingFormat.HEADER + "\n" + rowWith(column, "NaN"));
            IOException error = assertThrows(IOException.class, () -> Recording.load(csv));
            assertTrue(error.getMessage().contains("Non-finite"));
        }
    }

    @Test public void rejectsBackwardsTime() throws IOException {
        File csv = file(RecordingFormat.HEADER + "\n" + rowWith(0, "1") + "\n" + ROW);
        assertTrue(assertThrows(IOException.class, () -> Recording.load(csv))
                .getMessage().contains("backwards"));
    }

    @Test public void rejectsInvalidMode() throws IOException {
        File csv = file(RecordingFormat.HEADER + "\n" + rowWith(12, "3"));
        assertTrue(assertThrows(IOException.class, () -> Recording.load(csv))
                .getMessage().contains("DriveMode"));
    }

    @Test public void rejectsMissingEmptyAndWrongFormatFiles() throws IOException {
        assertThrows(IOException.class, () -> Recording.load(new File(temp.getRoot(), "missing.csv")));
        for (String content : new String[]{"", RecordingFormat.HEADER, "old,header\n1,2",
                RecordingFormat.HEADER + "\n1,2"}) {
            File csv = file(content);
            assertThrows(IOException.class, () -> Recording.load(csv));
        }
    }
}
