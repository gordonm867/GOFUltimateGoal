package org.firstinspires.ftc.teamcode.GOFUltimateGoal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Wobble;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name="GOFTeleOp",group="GOF")
public class GOFTeleOp extends MyOpMode {
    private     ArrayList<Subsystem>    subsystems = new ArrayList<>();
    private     Drivetrain              drive;
    private     Intake                  intake;
    private     GOFHardware             robot      = GOFHardware.getInstance();
    private     Odometry                odometry;
    private     Shooter                 shooter;
    private     Wobble                  wobble;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap);
        drive = new Drivetrain(Subsystem.State.OFF);
        intake = new Intake(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        shooter = new Shooter(Subsystem.State.OFF);
        wobble = new Wobble(Subsystem.State.OFF);

        robot.enabled = true;

        subsystems.add(odometry);
        subsystems.add(drive);
        subsystems.add(intake);
        subsystems.add(shooter);
        subsystems.add(wobble);

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
    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
