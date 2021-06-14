package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;

import java.io.IOException;

@Autonomous(name="PARK-BLUE")
public class PARKBLUE extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    Drivetrain drive;
    Shooter shooter;
    Wobble wobble;
    Odometry odometry;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        Globals.AUTO = true;
        robot.init(hardwareMap, telemetry);
        odometry = Odometry.getInstance(robot);
        robot.led.close();
        robot.resetOmnis();
        drive = new Drivetrain(Subsystem.State.OFF);
        shooter = new Shooter(Subsystem.State.OFF);
        wobble = new Wobble(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        odometry.reset();
        robot.resetOmnis();
        odometry.reset();
        robot.enabled = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        double displacement = odometry.getPoint().distance(new Point(odometry.getX(), -0.5), Unit.FEET);
        double x = odometry.getX();
        double angle = odometry.getAngle();
        while(displacement > 0.1 || Math.abs(angle - 96) > 1) {
            displacement = odometry.getPoint().distance(new Point(odometry.getX(), -0.5), Unit.FEET);
            drive.update(robot, new Point(x, -0.5), odometry, 96, angle, robot.bulkRead());
            angle = odometry.getAngle();
        }
        robot.setDrivePower(0, 0, 0, 0);
        shooter.shot = false;
        while(!shooter.shot) {
            shooter.forceshoot(robot, Shooter.firstshotvel, false);
        }
        robot.shoot1.setPower(0);
        robot.shoot2.setPower(0);
        displacement = odometry.getPoint().distance(new Point(odometry.getX(), 0), Unit.FEET);
        x = odometry.getX();
        angle = odometry.getAngle();
        while(displacement > 0.1) {
            displacement = odometry.getPoint().distance(new Point(odometry.getX(), 0), Unit.FEET);
            drive.update(robot, new Point(x, 0), odometry, Double.NaN, angle, robot.bulkRead());
            angle = odometry.getAngle();
        }
        robot.setDrivePower(0, 0, 0, 0);
        requestOpModeStop();
    }
}
