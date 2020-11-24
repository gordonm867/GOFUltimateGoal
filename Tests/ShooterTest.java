package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Handler;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;

@TeleOp(name="ShooterTest")
public class ShooterTest extends MyOpMode {
    GOFHardware robot = GOFHardware.getInstance();
    Handler handler = Handler.getInstance();

    double velocity = 0;

    boolean xpressed = false;
    boolean ypressed = false;

    public void initOp() {
        robot.init(hardwareMap);
        Shooter shooter = new Shooter(Shooter.State.ON);
        handler.pushData("stv", 0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    public void loopOp() {
        if(gamepad1.x && !xpressed) {
            xpressed = true;
            velocity += 0.1;
        }
        if(!gamepad1.x) {
            xpressed = false;
        }
        if(gamepad1.y && !ypressed) {
            ypressed = true;
            velocity -= 0.1;
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
