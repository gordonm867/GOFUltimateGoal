package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.IOException;

@Disabled
@Config
@TeleOp(name="DualServoTest", group="Tests")
public class DualServoTest extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();

    public static double servo1pos = 0.5;
    public static double servo2pos = 0.5;
    public static boolean paused = false;
    public static boolean intake = false;
    public static boolean outtake = false;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        if(!paused) {
            robot.d1.setPosition(servo1pos);
            robot.d2.setPosition(servo2pos);
        }
        if(intake) {
            robot.setIntakePower(-Globals.MAX_IN_SPEED);
        }
        if(outtake) {
            robot.setIntakePower(Globals.MAX_IN_SPEED);
        }
        if(!intake && !outtake) {
            robot.setIntakePower(0);
        }
        telemetry.addData("d1", robot.d1.getPosition());
        telemetry.addData("d2", robot.d2.getPosition());
        telemetry.update();
    }
}
