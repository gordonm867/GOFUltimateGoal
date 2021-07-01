package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Handler;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Intake;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.TeleOpPipeline;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
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

        robot.enabled = true;

        subsystems.add(odometry);
        subsystems.add(intake);
        subsystems.add(shooter);
        subsystems.add(wobble);
        robot.cameraInit(new TeleOpPipeline());

        Shooter.off = true;

        targX = odometry.getX();
        targY = odometry.getY();
        targA = odometry.getAngle();
    }

    public void loopOp() {
        if(robot.pipeline instanceof TeleOpPipeline && !((TeleOpPipeline) robot.pipeline).rects.isEmpty()) {
            Rect rect = Imgproc.boundingRect(((TeleOpPipeline) robot.pipeline).rects.get(0));
            telemetry.addData("Size", ((TeleOpPipeline) robot.pipeline).rects.get(0).size().area());
            telemetry.addData("Predicted Angle", Functions.normalize(Math.toDegrees(Math.atan2(rect.y + 480, rect.x - 320)) + odometry.getAngle()));
        }
        telemetry.addData("Distance", Math.sqrt(Math.pow(odometry.getX() - 3, 2) + Math.pow(odometry.getY() - 6, 2)));
        telemetry.update();
        Globals.MAX_SPEED = 0.5;
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, odometry.getAngle(),data, data2, odometry);
        }
        double displacement = odometry.getPoint().distance(new Point(targX, targY), Unit.FEET);
        double angle = odometry.getAngle() - targA;
        if(displacement > 3.0/96.0 || Math.abs(angle) > 0.5) {
            drive.update(robot, new Point(targX, targY), odometry, targA, odometry.getAngle(), data);
        }
        else {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }
}