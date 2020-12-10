package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

public class Shooter implements Subsystem {

    private State state;

    boolean apressed = false;
    boolean rt = false;
    boolean lt = false;
    boolean g2 = true;

    boolean shooting = false;
    Target targ;

    double time = 0;
    int step = 0;
    int attempts = 0;

    double v = 0;
    double t = 0;

    public Shooter(State state) {
        this.state = state;
        handler.pushData("gamepad2", g2);
    }

    public enum Target {
        GOAL,
        POWER
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
        if(gamepad2.right_trigger > 0.05 && !rt) {
            rt = true;
            attempts = 0;
            shooting = true;
            targ = Target.GOAL;
        }
        if(gamepad2.right_trigger < 0.05 && rt) {
            rt = false;
        }
        if(gamepad2.left_trigger > 0.05 && !lt) {
            lt = true;
            attempts = 0;
            shooting = true;
            targ = Target.POWER;
        }
        if(gamepad2.left_trigger < 0.05) {
            lt = false;
        }
        if(robot.shoot1 != null && handler.contains("stv")) {
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
        }
        if(robot.shoot2 != null && handler.contains("stv")) {
            t = (-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5);
            ((DcMotorEx) robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            handler.pushData("sav", v);
        }
        if(gamepad2.a && !apressed) {
            apressed = true;
            g2 = !g2;
        }
        if(!gamepad2.a) {
            apressed = false;
        }
        if(shooting && Math.abs(v - t) < 0.85) {
            shoot(targ, robot);
        }
        else if(shooting) {
            handler.pushData("stv", -21.14375);
        }
        if(!shooting) {
            handler.pushData("stv", 0);
        }
        handler.pushData("gamepad2", g2);
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }

    public void shoot(Target target, GOFHardware robot) {
        attempts++;
        if(attempts == 3) {
            shooting = false;
            return;
        }
        if(step == 0 && robot.flicker != null) {
            robot.flicker.setPosition(0.65);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 1 && System.currentTimeMillis() - time > 200) {
            robot.flicker.setPosition(0.3);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > 200) {
            shoot(target, robot);
        }
    }
}
