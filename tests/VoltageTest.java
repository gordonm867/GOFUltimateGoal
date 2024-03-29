package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Intake;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name="VoltageIntakeTest", group="Tests")
public class VoltageTest extends MyOpMode {

    private     ArrayList<Subsystem>    subsystems = new ArrayList<>();
    private     Drivetrain              drive;
    private     Intake                  intake;
    private     GOFHardware             robot      = GOFHardware.getInstance();
    private     Odometry                odometry;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap, telemetry);
        drive = new Drivetrain(Subsystem.State.OFF);
        intake = new Intake(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);

        robot.enabled = true;

        robot.d1.setPosition(0.5);
        robot.d2.setPosition(0);

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
        for(Subsystem subsystem : subsystems) subsystem.update(gamepad1, gamepad2, robot, odometry.getAngle(),data, data2, odometry);
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
