package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Handler;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Intake;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.TeleDrive;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Disabled
@TeleOp(name="TeleDriveTeleOp",group="GOF")
public class TeleDriveTeleOp extends TeleDrive {
    private     ArrayList<Subsystem>    subsystems  = new ArrayList<>();
    private     Drivetrain              drive;
    private     Intake                  intake;
    private     GOFHardware             robot       = GOFHardware.getInstance();
    private     Odometry                odometry;
    private     Shooter                 shooter;
    private     Wobble                  wobble;
    private     Handler                 handler     = Handler.getInstance();

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap, telemetry);
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
        super.initOp();
    }

    public void loopOp() {
        super.loopOp();
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, odometry.getAngle(), data, data2, odometry);
        }
    }

    public void stopOp() {
        super.stopOp();
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
