package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Handler;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
@Disabled
@Config
@TeleOp(name="ShooterTest", group="Tests")
//@Disabled
public class ShooterTest extends MyOpMode {
    GOFHardware robot = GOFHardware.getInstance();
    Handler handler = Handler.getInstance();

    double velocity = 0;

    boolean xpressed = false;
    boolean ypressed = false;

    private ArrayList<Subsystem> subsystems = new ArrayList<>();
    //private Drivetrain drive;
    //private Intake intake;
    private Odometry odometry;
    private Shooter shooter;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap, telemetry);

        //drive = new Drivetrain(Subsystem.State.OFF);
        //intake = new Intake(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        shooter = new Shooter(Shooter.State.ON);
        robot.enabled = true;

        subsystems.add(odometry);
        //subsystems.add(drive);
        //subsystems.add(intake);
        subsystems.add(shooter);

        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }

        handler.pushData("" + "stv", 0.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    public void loopOp() {
        double increment = 0.1;
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, odometry.getAngle(),data, data2, odometry);
        }
        if(gamepad1.x && !xpressed) {
            xpressed = true;
            velocity += increment;
            if(Math.abs(velocity) < 15) {
                if(velocity >= 0) {
                    velocity = 15;
                }
                else {
                    velocity = 0;
                }
            }
        }
        if(!gamepad1.x) {
            xpressed = false;
        }
        if(gamepad1.y && !ypressed) {
            ypressed = true;
            velocity -= increment;
            if(Math.abs(velocity) < 15) {
                if(velocity <= 0) {
                    velocity = -15;
                }
                else {
                    velocity = 0;
                }
            }
        }
        if(!gamepad1.y) {
            ypressed = false;
        }
        handler.pushData("stv", velocity);
        telemetry.addData("Target velocity", velocity);
        if(handler.contains("sav")) {
            telemetry.addData("Actual velocity", handler.getData("sav"));
        }
        telemetry.update();
    }
}
