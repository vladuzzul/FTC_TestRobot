package org.firstinspires.ftc.teamcode.testrobot.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;

/** Optional intake: all command factories reserve this mechanism. */
public final class Intake implements Mechanism {
    private DcMotorEx motor;

    public void initialize(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        stopMotor();
    }

    public Command on(boolean reversed) {
        return instant(() -> motor.setPower(reversed ? -Constants.INTAKE_POWER : Constants.INTAKE_POWER));
    }

    public Command off() { return instant(this::stopMotor); }

    public Command toggle(boolean reversed) {
        return instant(() -> motor.setPower(isOn() ? 0
                : (reversed ? -Constants.INTAKE_POWER : Constants.INTAKE_POWER)));
    }

    public boolean isOn() { return motor != null && Math.abs(motor.getPower()) > 0; }

    public void stopMotor() { if (motor != null) motor.setPower(0); }

    public void release() { stopMotor(); motor = null; }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake power", motor == null ? 0 : motor.getPower());
    }
}
