package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Handler;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Intake;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.openftc.revextensions2.RevBulkData;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

@TeleOp(name="GOFTeleOp",group="GOF")
public class GOFTeleOp extends MyOpMode {
    private     ArrayList<Subsystem>    subsystems  = new ArrayList<>();
    private     Drivetrain              drive;
    private     Intake                  intake;
    private     GOFHardware             robot       = GOFHardware.getInstance();
    private     Odometry                odometry;
    private     Shooter                 shooter;
    private     Wobble                  wobble;
    private     Handler                 handler     = Handler.getInstance();

    public boolean red = true;

    private double lastangle = 0;
    private double lasttime = 0;

    public double time = 0;

    public double max = 0;

    public void initOp() {
        Globals.AUTO = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.MAX_SPEED = 1.0;
        Globals.MIN_SPEED = 0.25;
        robot.init(hardwareMap, telemetry);
        robot.wobble.setPosition(Wobble.openpose);
        odometry = Odometry.getInstance(robot);
        drive = new Drivetrain(Subsystem.State.OFF);
        intake = new Intake(Subsystem.State.OFF);
        shooter = new Shooter(Subsystem.State.OFF);
        wobble = new Wobble(Subsystem.State.OFF);

        robot.enabled = true;

        subsystems.add(odometry);
        subsystems.add(drive);
        subsystems.add(intake);
        subsystems.add(shooter);
        subsystems.add(wobble);

        RevBulkData data = robot.bulkRead();
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
        try {
            BufferedReader read = new BufferedReader(new FileReader(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt")));
            double angle = Double.parseDouble(read.readLine());
            odometry.load(angle);
            //10, 3, 0, 0
            telemetry.addData("PIDF", ((DcMotorEx)robot.shoot1).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("lastAngle", Odometry.lastAngle);
            telemetry.addData("Handler angle", handler.getData("Angle"));
            telemetry.addData("Point", odometry.getPoint());
            telemetry.addData("Odometry angle", odometry.getAngle());
            telemetry.addData("Text file angle", angle);
            telemetry.addData("xraw", data.getMotorCurrentPosition(robot.rf));
            telemetry.addData("yraw", data.getMotorCurrentPosition(robot.rb));
            try {
                BufferedReader read2 = new BufferedReader(new FileReader(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/color.txt")));
                String color = read2.readLine();
                if(color.equalsIgnoreCase("blue")) {
                    red = false;
                }
                else {
                    red = true;
                }
                handler.pushData("Color", red ? "Red" : "Blue");
                telemetry.addData("Color", red ? "Red" : "Blue");
            }
            catch(Exception e) {
                handler.pushData("Color", red ? "Red" : "Blue");
                telemetry.addData("Color", red ? "Red" : "Blue");
            }
            telemetry.update();
        }
        catch(Exception e) {
            telemetry.addData("uh", e.getMessage());
            telemetry.update();
        }
        lastangle = odometry.getAngle();
        lasttime = System.currentTimeMillis();
    }

    public void startOp() {
        time = System.currentTimeMillis();
        Odometry.angleOffset = -odometry.getAngle() + Globals.START_THETA;
    }

    public void loopOp() {
        double angle = odometry.getAngle();
        if(angle != lastangle || System.currentTimeMillis() - lasttime >= 200) {
            double omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000);
            handler.pushData("Omega", omega);
            lasttime = System.currentTimeMillis();
            lastangle = angle;
        }
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for (Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, angle, data, data2, odometry);
        }
        if(robot.led != null) {
            if(gamepad2.left_stick_y < 0) {
                robot.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
            }
            else {
                robot.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
        }
        telemetry.addData("We will shoot at", Shooter.firstshotvel);
        telemetry.addData("Angle", angle);
        telemetry.addData("Max", max);
        if(handler.contains("stv")) {
            telemetry.addData("Target", (double)handler.getData("stv"));
        }
        if(handler.contains("sav")) {
            telemetry.addData("Shooter velocity", (double)handler.getData("sav"));
        }
        if(handler.contains("saa")) {
            telemetry.addData("Shooter acceleration", Math.abs((double)handler.getData("saa")));
        }
        telemetry.addData("Timing", (shooter.endshootingtimer - shooter.beginshootingtimer) / 1000.0);
        telemetry.update();
    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
