package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name="VoltageIntakeTest")
public class VoltageTest extends MyOpMode {

    private     ArrayList<Subsystem>    subsystems = new ArrayList<>();
    private     Drivetrain              drive;
    private     Intake                  intake;
    private     GOFHardware             robot      = GOFHardware.getInstance();
    private     Odometry                odometry;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap);
        drive = new Drivetrain(Subsystem.State.OFF);
        intake = new Intake(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);

        robot.enabled = true;

        subsystems.add(odometry);
        subsystems.add(drive);
        subsystems.add(intake);

        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
    }

    public void loopOp() {
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, data, data2, odometry);
        }
        telemetry.addData("lf", ((ExpansionHubMotor)robot.lf).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("rf", ((ExpansionHubMotor)robot.rf).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("rb", ((ExpansionHubMotor)robot.rb).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("lb", ((ExpansionHubMotor)robot.lb).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.addData("in", ((ExpansionHubMotor)robot.in).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        telemetry.update();
    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
