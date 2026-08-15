package org.firstinspires.ftc.teamcode.testrobot.utils.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Intake {
    private DcMotorEx intakeMotor;

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
    }
}
