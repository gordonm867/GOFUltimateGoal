package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.IOException;

@Disabled
@Config
@TeleOp(name="PureSensorTest", group="Tests")
public class PureSensorTest extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();

    double timer = System.currentTimeMillis() - 2000;
    boolean updated = false;

    public static double SPEED = 0;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot.init(hardwareMap, telemetry);
        robot.in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.in2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.d1.setPosition(0.68);
        robot.d2.setPosition(0.4);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        robot.setIntakePower(-SPEED);
        double distance = robot.ringsensor.getDistance(DistanceUnit.MM);
        if(distance < 29) {
            timer = System.currentTimeMillis();
            updated = false;
        }
        if(System.currentTimeMillis() - timer < 2000) {
            telemetry.addData("OH MY!", "RING SPOTTED!");
            telemetry.update();
        }
        else if(!updated) {
            updated = true;
            telemetry.addData("", "");
            telemetry.update();
        }
        if(gamepad2.b) {
            timer = System.currentTimeMillis() - 2000;
        }
    }
}
