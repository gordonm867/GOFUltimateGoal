package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;
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
    public boolean readying = false;
    Target targ;

    double time = 0;
    int step = 0;
    public int attempts = 0;

    public double v = 0;
    public double t = 0;
    public double a = 0;

    public double v2 = 0;
    public double a2 = 0;

    public double lasttime = System.currentTimeMillis();
    public double lasttime2 = System.currentTimeMillis();
    public double lastv = 0;
    public double lastv2 = 0;

    public static double shootTime = 750.0/9.0;
    public static double shootIn = 0.34;
    public static double shootOut = 0.55;

    public static double p = 60;
    public static double i = 3;
    public static double d = 40;
    public static double f = 0;

    public static double vel = 16.8;
    public static double oldvel = 16.8;

    public static int thing = 4;
    public static int oldthing = 4;

    public boolean uh = false;
    public boolean xpressed = false;
    public int powershots = 0;

    public double cycles = 0;

    public int shots = 0;

    public static double maxvel = 18;

    public Shooter(State state) {
        this.state = state;
        handler.pushData("gamepad2", g2);
    }

    public enum Target {
        GOAL,
        POWER
    }

    public enum Powerstate {
        IDLE,
        FIRST,
        FIRSTDONE,
        SECOND,
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, double angle, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
        handler.pushData("Power Shots", powershots);
        if(!uh && (handler.contains("Power State") && (handler.getData("Power State") == Drivetrain.Powerstate.FIRST || handler.getData("Power State") == Drivetrain.Powerstate.SECOND || handler.getData("Power State") == Drivetrain.Powerstate.THIRD) || gamepad2.y)) {
            uh = true;
            thing = 2;
            rt = true;
            vel = 15.2;
            attempts = 0;
            shooting = true;
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.POWER;
        }
        if(gamepad2.right_trigger > 0.05 && !rt) {
            shots++;
            if(shots % 3 == 0) {
                odometry.shootreset();
            }
            thing = oldthing;
            rt = true;
            Point target;
            if(handler.contains("Color") && handler.getData("Color").toString().equalsIgnoreCase("Blue")) {
                target = new Point(-3, 6);
            }
            else {
                target = new Point(3, 6);
            }
            vel = (((maxvel - oldvel) / 4) * (odometry.getPoint().distance(target, Unit.FEET) - 10)) + maxvel;
            attempts = 0;
            shooting = true;
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.GOAL;
        }
        if(gamepad2.x && !xpressed) {
            xpressed = true;
            if(readying) {
                readying = false;
            }
            else {
                readying = true;
                start(robot, 15.15);
            }
        }
        if(!gamepad2.x) {
            xpressed = false;
        }
        if(gamepad2.right_trigger < 0.05 && rt) {
            rt = false;
        }
        if(gamepad2.left_trigger > 0.05 && !lt) {
            readying = true;
            vel = oldvel;
            start(robot, vel - 0.15);
        }
        if(gamepad2.left_trigger < 0.05) {
            lt = false;
        }
        if(gamepad2.left_bumper) {
            shooting = false;
            ready = false;
            readying = false;
            robot.flicker.setPosition(shootOut);
        }
        if(!handler.contains("stv")) {
            handler.pushData("stv", 0.0);
        }
        if(robot.shoot1 != null) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            v2 = (((DcMotorEx)robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime2 = (System.currentTimeMillis() - lasttime2) / 1000.0;
            if(deltatime2 > 0.02) {
                a2 = (v2 - lastv2) / (deltatime2);
                lastv2 = v2;
            }
            lasttime2 = System.currentTimeMillis();
            handler.pushData("sav2", v2);
        }
        if(robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            if(deltatime > 0.02) {
                a = (v - lastv) / (deltatime);
                lastv = v;
                lasttime = System.currentTimeMillis();
            }
            handler.pushData("sav", v);
        }
        if(gamepad2.a && !apressed) {
            apressed = true;
            g2 = !g2;
        }
        if(!gamepad2.a) {
            apressed = false;
        }
        if(shooting && (ready || (Math.abs(Math.abs(v) - Math.abs(t)) < 0.15) && Math.abs(a) < 0.5)) {
            ready = true;
            shoot(targ, robot);
        }
        else if(shooting) {
            handler.pushData("stv", vel);
        }
        if(!shooting && !readying) {
            handler.pushData("stv", 0.0);
        }
        handler.pushData("gamepad2", g2);
    }

    public void shoot(GOFHardware robot, double velocity, boolean once) {
        if(Math.abs(velocity) < Math.abs(oldvel)) {
            ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(120, 5, 10, 0);
            ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(120, 5, 10, 0);
        }
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            v2 = (((DcMotorEx)robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime2 = (System.currentTimeMillis() - lasttime2) / 1000.0;
            if(deltatime2 > 0.02) {
                a2 = (v2 - lastv2) / (deltatime2);
                lastv2 = v2;
            }
            handler.pushData("sav2", v2);
        }
        if(robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            if(deltatime > 0.02) {
                a = (v - lastv) / (deltatime);
                lastv = v;
                lasttime = System.currentTimeMillis();
            }
            handler.pushData("sav", v);
        }
        if(/*ready ||*/ (Math.abs(Math.abs(v) - Math.abs(t)) < 0.1 && Math.abs(a) < 1) && (Math.abs(Math.abs(v2) - Math.abs(t)) < 0.1 && Math.abs(a2) < 1)) {
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
        if(Math.abs(velocity) < Math.abs(oldvel)) {
            ((DcMotorEx) robot.shoot1).setVelocityPIDFCoefficients(120, 5, 10, 0);
            ((DcMotorEx) robot.shoot2).setVelocityPIDFCoefficients(120, 5, 10, 0);
        }
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null) {
            ((DcMotorEx)robot.shoot1).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
        }
        if(robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            ((DcMotorEx)robot.shoot2).setVelocity((-((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            if(deltatime > 0.02) {
                a = (v - lastv) / (deltatime);
                lastv = v;
                lasttime = System.currentTimeMillis();
            }
            handler.pushData("sav", v);
        }
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }

    public void shoot(Target target, GOFHardware robot) {
        if(Math.abs(vel) < Math.abs(oldvel) || target == Target.POWER) {
            ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(120, 5, 10, 0);
            ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(120, 5, 10, 0);
        }
        if(step == 0 && robot.flicker != null) {
            attempts++;
            if(attempts == thing) {
                if(Math.abs(vel) < Math.abs(oldvel)) {
                    uh = false;
                    powershots++;
                }
                shooting = false;
                if(target != Target.POWER) {
                    readying = false;
                }
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
        if(step == 2 && System.currentTimeMillis() - time > shootTime * 2) {
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
