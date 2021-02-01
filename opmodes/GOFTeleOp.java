package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import android.os.Environment;

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

    public boolean lt = false;
    public boolean red = true;

    public double lastangle = 90;
    public double lasttime = System.currentTimeMillis();

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        Globals.MIN_SPEED = 0.25;
        robot.init(hardwareMap);
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
        RevBulkData data2 = robot.bulkReadTwo();
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
    }

    public void loopOp() {
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(Shooter.p, Shooter.i, Shooter.d, Shooter.f);
        ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(Shooter.p, Shooter.i, Shooter.d, Shooter.f);
        double angle = odometry.getAngle();
        double omega  = (odometry.getAngle() - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
        handler.pushData("Omega", omega);
        lastangle = angle;
        lasttime = System.currentTimeMillis();
        for (Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, angle, data, data2, odometry);
        }
        telemetry.addData("Angle", odometry.getAngle());
        telemetry.addData("Omega", omega);
        telemetry.update();

    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
