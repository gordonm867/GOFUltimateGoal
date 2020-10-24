package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name="TestleOp",group="Tests")
public class TestleOp extends MyOpMode {
    private ArrayList<Subsystem> subsystems = new ArrayList<>();
    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;

    boolean apressed = false;
    boolean running = false;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap);
        robot.resetOmnis();
        drive = new Drivetrain(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        odometry.reset();

        robot.enabled = true;
        subsystems.add(drive);
        subsystems.add(odometry);

        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
    }

    public void loopOp() throws InterruptedException {
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        if(!running) {
            Globals.MAX_SPEED = 1.0;
            for (Subsystem subsystem : subsystems) {
                subsystem.update(gamepad1, gamepad2, robot, data, data2, odometry);
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getHOmniPos(data));
                telemetry.addData("yraw", robot.getVOmniPos(data));
                telemetry.addData("angle", odometry.getAngle());
                telemetry.update();
            }
        }
        else {
            Globals.MAX_SPEED = 0.4;
            Point target = new Point(2, 2);
            double displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            while(displacement > 0.25) {
                drive.update(robot, target, odometry, 135, odometry.getAngle(), data);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getHOmniPos(data));
                telemetry.addData("yraw", robot.getVOmniPos(data));
                telemetry.update();
            }
            double time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 5000) {
                odometry.update(data);
                telemetry.addData("Status", "Waiting.... ( " + (((5000 - (System.currentTimeMillis() - time))) / 1000.0) + " seconds left");
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getHOmniPos(data));
                telemetry.addData("yraw", robot.getVOmniPos(data));
                telemetry.update();
                if(!opModeIsActive()) {
                    throw new InterruptedException();
                }
            }
            Globals.MAX_SPEED = 1.0;
            target = new Point(2, -4);
            displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            while(displacement > 0.25) {
                drive.update(robot, target, odometry, 0, odometry.getAngle(), data);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getHOmniPos(data));
                telemetry.addData("yraw", robot.getVOmniPos(data));
                telemetry.update();
            }
            telemetry.addData("x", odometry.getX());
            telemetry.addData("y", odometry.getY());
            telemetry.addData("xraw", robot.getHOmniPos(data));
            telemetry.addData("yraw", robot.getVOmniPos(data));
            telemetry.update();
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 3000) {
                odometry.update(data);
                if(!opModeIsActive()) {
                    throw new InterruptedException();
                }
            }
            running = false;
        }
        if(gamepad1.x && !apressed) {
            apressed = true;
            running = true;
        }
        if(!gamepad1.x) {
            apressed = false;
        }
    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
