package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

public class Shooter implements Subsystem {

    private State state;

    public Shooter(State state) {
        this.state = state;
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
        if(robot.shoot1 != null && handler.contains("stv")) {
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI), AngleUnit.DEGREES);
        }
        if(robot.shoot2 != null && handler.contains("stv")) {
            ((DcMotorEx) robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI), AngleUnit.DEGREES);
            handler.pushData("sav", (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES)) * 4 * Math.PI * 0.0254 / 360.0);
        }
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }
}
