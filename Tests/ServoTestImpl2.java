package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;

@TeleOp(name="wobbletest")
@Disabled
public class ServoTestImpl2 extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    ServoTest servoTest;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap);
        servoTest = new ServoTest(robot.wobble, telemetry);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException {
        servoTest.run(gamepad1);
    }
}
