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

//@Disabled
@Config
@TeleOp(name="WobbleTurretTest")
public class WobbleTurretTest extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();

    public static double servo1pos = 0.5;
    public static double servo2pos = 0.5;
    public static double servo3pos = 0.5;
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
            robot.w1.setPosition(Globals.W1POS);
            robot.w2.setPosition(Globals.W2POS);
            robot.wobble.setPosition(servo3pos);
        }
        telemetry.addData("w1", robot.w1.getPosition());
        telemetry.addData("w2", robot.w2.getPosition());
        telemetry.addData("claw", robot.wobble.getPosition());
        telemetry.update();
    }
}
