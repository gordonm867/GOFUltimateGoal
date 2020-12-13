package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;

@TeleOp(name="flickertest")
public class ServoTestImpl extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    ServoTest servoTest;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap);
        servoTest = new ServoTest(robot.flicker, telemetry);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException {
        servoTest.run(gamepad1);
    }
}
