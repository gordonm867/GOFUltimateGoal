package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter implements Subsystem {

    private State state;

    boolean apressed = false;
    boolean rt = false;
    boolean lt = false;
    boolean g2 = true;

    boolean ready = false;

    public boolean shot = false;

    public boolean shooting = false;
    Target targ;

    double time = 0;
    int step = 0;
    public int attempts = 0;

    public double v = 0;
    public double t = 0;
    public double a = 0;

    public double lasttime = System.currentTimeMillis();
    public double lastv = 0;

    public static double shootTime = 50.0;
    public static double shootIn = 0.35;
    public static double shootOut = 0.57;

    public static double vel = 17.3;

    public static int thing = 5;

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
            thing = 5;
            rt = true;
            vel = 17.8;
            attempts = 0;
            shooting = true;
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.GOAL;
        }
        if(gamepad2.right_trigger < 0.05 && rt) {
            rt = false;
        }
        if(gamepad2.left_trigger > 0.05 && !lt) {
            thing = 2;
            lt = true;
            vel = 15.5;
            attempts = 0;
            shooting = true;
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.GOAL;
        }
        if(gamepad2.left_trigger < 0.05) {
            lt = false;
        }
        if(gamepad2.left_bumper) {
            shooting = false;
            ready = false;
            robot.flicker.setPosition(shootOut);
        }
        if(robot.shoot1 != null && handler.contains("stv")) {
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
        }
        if(robot.shoot2 != null && handler.contains("stv")) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            lastv = v;
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            a = (v - lastv) / (deltatime);
            lasttime = System.currentTimeMillis();
            handler.pushData("sav", v);
        }
        if(gamepad2.a && !apressed) {
            apressed = true;
            g2 = !g2;
        }
        if(!gamepad2.a) {
            apressed = false;
        }
        if(shooting && ((Math.abs(Math.abs(v) - Math.abs(t)) < 0.1 && Math.abs(a) < 0.05) /* || ready */)) {
            ready = true;
            shoot(targ, robot);
        }
        else if(shooting) {
            handler.pushData("stv", vel);
        }
        if(!shooting) {
            handler.pushData("stv", 0.0);
        }
        handler.pushData("gamepad2", g2);
    }

    public void shoot(GOFHardware robot, double velocity, boolean once) {
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null) {
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
        }
        if(robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            lastv = v;
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            a = (v - lastv) / (deltatime);
            lasttime = System.currentTimeMillis();
            handler.pushData("sav", v);
        }
        if(/*ready || */(Math.abs(Math.abs(v) - Math.abs(t)) < 0.1 && Math.abs(a) < 0.05)) {
            ready = true;
            if(once) {
                shootonce(targ, robot);
            }
            else {
                shoot(targ, robot);
            }
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
            lastv = v;
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            a = (v - lastv) / (deltatime);
            lasttime = System.currentTimeMillis();
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
            if(attempts == thing) {
                shooting = false;
                ready = false;
                return;
            }
            robot.flicker.setPosition(shootIn);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 1 && System.currentTimeMillis() - time > shootTime) {
            robot.flicker.setPosition(shootOut);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > shootTime * 1.5) {
            step = 0;
            shoot(target, robot);
        }
    }

    public void shootonce(Target target, GOFHardware robot) {
        if(step == 0 && robot.flicker != null) {
            robot.flicker.setPosition(shootIn);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 1 && System.currentTimeMillis() - time > shootTime) {
            robot.flicker.setPosition(shootOut);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > shootTime) {
            step = 0;
            shot = true;
        }
    }
}
