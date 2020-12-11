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

    public boolean shot = false;

    boolean shooting = false;
    Target targ;

    double time = 0;
    int step = 0;
    int attempts = 0;

    public double v = 0;
    public double t = 0;

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
            handler.pushData("stv", 19.2);
            t = (double)handler.getData("stv");
            targ = Target.GOAL;
        }
        if(gamepad2.right_trigger < 0.05 && rt) {
            rt = false;
        }
        if(gamepad2.left_trigger > 0.05 && !lt) {
            lt = true;
            attempts = 0;
            shooting = true;
            handler.pushData("stv", 19.2);
            t = (double)handler.getData("stv");
            targ = Target.POWER;
        }
        if(gamepad2.left_trigger < 0.05) {
            lt = false;
        }
        if(robot.shoot1 != null && handler.contains("stv")) {
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
        }
        if(robot.shoot2 != null && handler.contains("stv")) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
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
        if(shooting && Math.abs(Math.abs(v) - Math.abs(t)) < 0.25) {
            shoot(targ, robot);
        }
        else if(shooting) {
            handler.pushData("stv", 19.0);
        }
        if(!shooting) {
            handler.pushData("stv", 0.0);
        }
        handler.pushData("gamepad2", g2);
    }

    public void shoot(GOFHardware robot, double velocity) {
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null) {
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
        }
        if(robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            handler.pushData("sav", v);
        }
        if(Math.abs(Math.abs(v) - Math.abs(t)) < 0.50) {
            shootonce(targ, robot);
        }
        handler.pushData("gamepad2", g2);
    }

    public void start(GOFHardware robot, double velocity) {
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null) {
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
        }
        if(robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            handler.pushData("sav", v);
        }
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }

    public void shoot(Target target, GOFHardware robot) {
        if(step == 0 && robot.flicker != null) {
            attempts++;
            if(attempts == 4) {
                shooting = false;
                return;
            }
            robot.flicker.setPosition(0.70);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 1 && System.currentTimeMillis() - time > 400) {
            robot.flicker.setPosition(0.25);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > 400) {
            step = 0;
            shoot(target, robot);
        }
    }

    public void shootonce(Target target, GOFHardware robot) {
        if(step == 0 && robot.flicker != null) {
            robot.flicker.setPosition(0.70);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 1 && System.currentTimeMillis() - time > 500) {
            robot.flicker.setPosition(0.2);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > 300) {
            step = 0;
            shot = true;
        }
    }
}
