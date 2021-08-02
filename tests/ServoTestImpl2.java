package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
@Disabled
@TeleOp(name="wobbletest", group="Tests")
public class ServoTestImpl2 extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    ServoTest servoTest;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap, telemetry);
        servoTest = new ServoTest(robot.wobble, telemetry);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException {
        servoTest.run(gamepad1);
    }
}
