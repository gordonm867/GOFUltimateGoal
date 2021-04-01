package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.IOException;

@Config
@TeleOp(name="ColorSensorTest")
public class ColorSensorTest extends MyOpMode {

    public static boolean intaking = false;
    public static boolean outtaking = false;

    GOFHardware robot = GOFHardware.getInstance();


    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        if(intaking) {
            robot.setIntakePower(-1.0);
        }
        else if(outtaking) {
            robot.setIntakePower(1.0);
        }
        else {
            robot.setIntakePower(0);
        }
        telemetry.addData("Reading", robot.ringsensor);
        telemetry.update();
    }
}
