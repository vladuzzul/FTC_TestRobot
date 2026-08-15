package org.firstinspires.ftc.teamcode.testrobot.utils.components;

import static org.firstinspires.ftc.teamcode.testrobot.utils.Constants.INTAKE_POWER;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class IntakeController {
    private DcMotorEx intakeMotor;

    private boolean isOn = false;

    public void telemetry(Telemetry telemetry){
        telemetry.addLine("--- INTAKE ---");
        telemetry.addData("Power:", "%.2f", intakeMotor.getPower());
    }

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setPower(0.0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
        isOn = abs(power) > 0;
    }

    public void turnOn(){
        intakeMotor.setPower(INTAKE_POWER);
        isOn = true;
    }

    public void turnOn(boolean reversed){
        if (reversed){
            intakeMotor.setPower(-INTAKE_POWER);
        }
        else {
            intakeMotor.setPower(INTAKE_POWER);
        }
        isOn = true;
    }

    public void turnOff(){
        intakeMotor.setPower(0);
        isOn = false;
    }

    public void toggleIntake(boolean reversed){
        isOn = !isOn;
        if (isOn) {
            turnOn(reversed);
        }
        else {
            turnOff();
        }
    }

    public boolean isOn(){
        return isOn;
    }
}
