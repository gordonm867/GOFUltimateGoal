package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;

@Config
@TeleOp(name="wobble")
public class WobbleTuner extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    public static int pos = 0;
    boolean xpressed = false;
    boolean ypressed = false;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap);
        robot.wobblewheel.setTargetPosition(pos);
        robot.wobblewheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobblewheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException {
        robot.wobblewheel.setTargetPosition(pos);
        robot.wobblewheel.setPower(Globals.MAX_WOBBLE);
        if(gamepad1.x && !xpressed) {
            xpressed = true;
            pos += 50;
        }
        if(!gamepad1.x) {
            xpressed = false;
        }
        if(gamepad1.y && !ypressed) {
            ypressed = true;
            pos -= 50;
        }
        if(!gamepad1.y) {
            ypressed = false;
        }
    }
}
