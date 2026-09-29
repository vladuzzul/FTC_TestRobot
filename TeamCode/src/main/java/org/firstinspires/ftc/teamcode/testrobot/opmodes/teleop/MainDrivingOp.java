package org.firstinspires.ftc.teamcode.testrobot.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.testrobot.Robot;
import org.firstinspires.ftc.teamcode.testrobot.opmodes.base.MainTeleOp;
import org.firstinspires.ftc.teamcode.testrobot.utils.Constants;

@NextTeleop(name = "Main Driving", group = Constants.MAIN_GROUP)
public final class MainDrivingOp extends MainTeleOp {
    public MainDrivingOp(Robot robot) { super(robot); }
}
