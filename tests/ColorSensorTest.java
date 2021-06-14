package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.IOException;

@Config
@TeleOp(name="ColorSensorTest")
public class ColorSensorTest extends MyOpMode {

    public static double intaking = 0;

    GOFHardware robot = GOFHardware.getInstance();


    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        robot.setIntakePower(-intaking);
        telemetry.addData("Distance", robot.ringsensor.getDistance(DistanceUnit.MM));
        NormalizedRGBA color = robot.ringsensor.getNormalizedColors();
        telemetry.addData("Color", color.red + ", " + color.green + ", " + color.blue + ", " + color.alpha);
        telemetry.update();
    }
}
