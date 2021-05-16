package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Handler;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Intake;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Config
@TeleOp(name="ShootingTune",group="GOF")
public class Tuning extends MyOpMode {
    private     ArrayList<Subsystem>    subsystems  = new ArrayList<>();
    private     Drivetrain              drive;
    private     Intake                  intake;
    private     GOFHardware             robot       = GOFHardware.getInstance();
    private     Odometry                odometry;
    private     Shooter                 shooter;
    private     Wobble                  wobble;
    private     Handler                 handler     = Handler.getInstance();

    public static double targX = Math.abs(Globals.START_X);
    public static double targY = Globals.START_Y;
    public static double targA = Globals.START_THETA;

    double lastangle;
    double lasttime = System.currentTimeMillis();
    double omega = 6;

    public void initOp() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.MAX_SPEED = 1.0;
        Shooter.thing = 4;
        robot.init(hardwareMap, telemetry);
        drive = new Drivetrain(Subsystem.State.OFF);
        intake = new Intake(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        shooter = new Shooter(Subsystem.State.OFF);
        wobble = new Wobble(Subsystem.State.OFF);

        robot.enabled = true;

        subsystems.add(odometry);
        subsystems.add(intake);
        subsystems.add(shooter);
        subsystems.add(wobble);

        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
        drive.setState(Subsystem.State.ON);
    }

    public void loopOp() {
        Globals.MAX_SPEED = 1.0;
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, odometry.getAngle(),data, data2, odometry);
        }
        double angle = odometry.getAngle();
        if(angle != lastangle) {
            double omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000);
            handler.pushData("Omega", omega);
            lasttime = System.currentTimeMillis();
            lastangle = angle;
        }
        telemetry.addData("Point", odometry.getPoint());
        telemetry.addData("Angle", odometry.getAngle());
        telemetry.addData("Integral", drive.integral);
        telemetry.addData("Target Angle", targA);
        double displacement = odometry.getPoint().distance(new Point(targX, targY), Unit.FEET);
        double angularerror = odometry.getAngle() - targA;
        if(displacement > 3.0/96.0 || Math.abs(angularerror) > 0.25 || Math.abs(odometry.getX() - targX) > 1.0/96.0 || omega > 0.15) {
            drive.update(robot, new Point(targX, targY), odometry, targA, odometry.getAngle(), data);
        }
        else {
            robot.setDrivePower(0, 0, 0, 0);
        }
        telemetry.update();
    }

    public void stopOp() {
        Shooter.thing = 4;
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}