package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

@Disabled
@TeleOp(name="Return")
public class Setup extends MyOpMode {

    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;
    Shooter shooter;
    Wobble wobble;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drivetrain(Subsystem.State.OFF);
        shooter = new Shooter(Subsystem.State.OFF);
        wobble = new Wobble(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        try {
            BufferedReader read = new BufferedReader(new FileReader(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt")));
            double angle = Double.parseDouble(read.readLine());
            odometry.load(angle);
        }
        catch(Exception e) {
            telemetry.addData("uh", e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        telemetry.addData("Angle", odometry.getAngle());
        telemetry.addData("Point", odometry.getPoint());
        telemetry.addData("Target", new Point(4.78125, -4.6));
        telemetry.update();
        drive.update(robot, new Point(4.78125, -4.6), odometry, 90, odometry.getAngle(), robot.bulkRead());
    }
}
