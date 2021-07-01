package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.IOException;

//@Disabled
@Config
@TeleOp(name="CameraServoTest", group="Tests")
public class CameraServoTest extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();

    public static double servo1pos = 0.5;
    public static double servo2pos = 0.5;
    public static boolean paused = false;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        if(!paused) {
            robot.c1.setPosition(servo1pos);
            robot.c2.setPosition(servo2pos);
        }
        telemetry.addData("c1", robot.c1.getPosition());
        telemetry.addData("c2", robot.c2.getPosition());
        telemetry.update();
    }
}
