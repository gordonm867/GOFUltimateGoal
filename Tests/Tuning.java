package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Handler;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Wobble;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;
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

    public static double targX = Globals.START_X;
    public static double targY = Globals.START_Y;
    public static double targA = Globals.START_THETA;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        Shooter.thing = 2;
        robot.init(hardwareMap);
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
        Globals.MAX_SPEED = 0.2;
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, data, data2, odometry);
        }
        drive.update(robot, new Point(targX, targY), odometry, targA, odometry.getAngle(), data2);
    }

    public void stopOp() {
        Shooter.thing = 4;
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
