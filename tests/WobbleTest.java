package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.IOException;

@Disabled
@Config
@TeleOp(name="WobbleTester", group="Tests")
public class WobbleTest extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();

    public static double w1pos = 0.65;
    public static double clawpos = Wobble.openpose;
    public static boolean paused = false;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        if(!paused) {
            robot.w1.setPosition(w1pos);
            robot.wobble.setPosition(clawpos);
        }
        telemetry.addData("w1", robot.w1.getPosition());
        telemetry.addData("claw", robot.wobble.getPosition());
        telemetry.update();
    }
}
