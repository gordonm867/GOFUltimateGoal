package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Circle;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Line;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Handler;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Intake;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.LocalizationPipeline;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;
import org.opencv.core.Rect;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
@Config
@TeleOp(name="CameraTest")
public class CameraTest extends MyOpMode {

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
        robot.cameraInit(new LocalizationPipeline());

        if(robot.pipeline instanceof LocalizationPipeline) {
            while (!isStarted() && !isStopRequested() && !((LocalizationPipeline) robot.pipeline).isProc) {
                telemetry.addData("Status", "Initializing OpenCV....");
                telemetry.update();
            }
        }
    }

    public void loopOp() {
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
        double size;
        ArrayList<Point> mypoints = new ArrayList<>();
        if(robot.pipeline instanceof LocalizationPipeline) {
            try {
                for (int x = 0; x < ((LocalizationPipeline) robot.pipeline).rects.size(); x++) {
                    Rect rect = ((LocalizationPipeline) robot.pipeline).rects.get(x).boundingRect();
                    size = rect.area();
                    double ang = Functions.normalize(Math.toDegrees(Math.atan2(rect.y + 480, rect.x - 320)) + odometry.getAngle() + 98);
                    double dist = ((1369.5873 * Math.pow(size, -0.41436702)) / 12.0);
                    double realx = ((1.5 / 12.0) * Math.sin(Math.toRadians(odometry.getAngle())) + ((4.5 / 12.0) * Math.cos(Math.toRadians(odometry.getAngle())))) + odometry.getX();
                    double realy = ((1.5 / 12.0) * Math.cos(Math.toRadians(odometry.getAngle())) + ((4.5 / 12.0) * Math.sin(Math.toRadians(odometry.getAngle())))) + odometry.getY();
                    ArrayList<Point> points = Functions.infiniteLineCircleIntersection(new Circle(new Point(realx, realy), dist), new Line(new Point(realx, realy), new Point(realx + 1, Math.tan(Math.toRadians(ang)))));
                    for (Point p : points) {
                        if (p.getY() > 0) {
                            //Point estimate = new Point(p.getX() + ((20.0/12) * Math.cos(Math.toRadians(odometry.getAngle()))), p.getY() + ((20.0/12) * Math.sin(Math.toRadians(odometry.getAngle()))));
                            mypoints.add(p);
                        }
                    }
                }
            }
            catch (Exception e) {}
        }
        StringBuilder telem = new StringBuilder();
        TelemetryPacket pack = new TelemetryPacket();
        Canvas field = pack.fieldOverlay();
        field = field.fillRect(targY * 12, -targX * 12, 18, 18);
        for(Point p : mypoints) {
            if(Math.abs(p.getY()) < 5.25 && Math.abs(p.getX()) < 5.25) {
                field = field.strokeCircle(p.getY() * 12, -p.getX() * 12, 5);
            }
        }
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }
}