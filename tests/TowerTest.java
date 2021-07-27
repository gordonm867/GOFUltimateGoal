package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.vision.UGAngleHighGoalPipeline;
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
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
@Config
@TeleOp(name="TowerTest", group="Tests")
public class TowerTest extends MyOpMode {

    private     ArrayList<Subsystem>    subsystems  = new ArrayList<>();
    private     Drivetrain              drive;
    private     Intake                  intake;
    private     GOFHardware             robot       = GOFHardware.getInstance();
    private     Odometry                odometry;
    private     Shooter                 shooter;
    private     Wobble                  wobble;
    private     Handler                 handler     = Handler.getInstance();

    private UGAngleHighGoalPipeline mypipeline = new UGAngleHighGoalPipeline(68);

    public static double targX = Math.abs(Globals.START_X);
    public static double targY = Globals.START_Y;
    public static double targA = Globals.START_THETA;

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

        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }

        robot.enabled = true;

        subsystems.add(drive);
        subsystems.add(odometry);
        subsystems.add(intake);
        subsystems.add(shooter);
        subsystems.add(wobble);
        robot.cameraInit(mypipeline );

        Shooter.off = true;

        targX = odometry.getX();
        targY = odometry.getY();
        targA = odometry.getAngle();
    }

    public void loopOp() {
        telemetry.addData("Pitch estimate", mypipeline.calculatePitch(UGAngleHighGoalPipeline.Target.RED));
        telemetry.addData("Yaw estimate", mypipeline.calculateYaw(UGAngleHighGoalPipeline.Target.RED));
        telemetry.addData("Angle", robot.gyro.getAngularOrientation().firstAngle);
        telemetry.addData("Distance", Math.sqrt(Math.pow(odometry.getX() - 3, 2) + Math.pow(odometry.getY() - 6, 2)));
        telemetry.update();
        Globals.MAX_SPEED = 0.5;
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, odometry.getAngle(),data, data2, odometry);
        }
    }
}