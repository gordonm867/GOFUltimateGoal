package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter implements Subsystem {

    private State state;

    boolean apressed = false;
    boolean rightpressed = false;
    boolean rt = false;
    boolean lt = false;
    boolean g2 = true;
    boolean uh = false;
    boolean xpressed = false;
    boolean dpadup = false;
    boolean dpaddown = false;
    boolean b = false;
    boolean bpressed = true;
    boolean ready = false;
    boolean b1pressed = false;

    public boolean shot = false;
    public boolean shooting = false;
    public boolean readying = false;

    public int powershots = 0;

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

    public static double shootTime = 65.0;
    public static double shootIn = 0.36;
    public static double shootOut = 0.55;

    public static double p = 70;
    public static double i = 1.5;
    public static double d = 0;
    public static double f = 13;
    public static double IP = 75;

    public static double PP = 30;
    public static double PI = 2;
    public static double PD = 0;
    public static double PF = 0;

    public static double vel = 16.14;
    public static double firstshotvel = 16.14;
    public static double secondshotvel = 16.14;
    public static double thirdshotvel = 16.14;

    public static double oldfirstshotvel = 16.14;
    public static double oldsecondshotvel = 16.14;
    public static double oldthirdshotvel = 16.14;

    public static int thing = 4;
    public static int oldthing = 4;

    public static double powershotvel = 14.5;

    public double cycles = 0;

    public int shots = 0;

    public static boolean waitingForDrive = false;

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
        if(!uh && (handler.contains("Power State") && (handler.getData("Power State") == Drivetrain.Powerstate.FIRST || handler.getData("Power State") == Drivetrain.Powerstate.SECOND || handler.getData("Power State") == Drivetrain.Powerstate.THIRD) || (gamepad2.y || Drivetrain.waitingForShoot))) {
            uh = true;
            thing = 2;
            vel = powershotvel;
            attempts = 0;
            shooting = true;
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.POWER;
        }
        if(uh && !gamepad1.y) {
            uh = false;
        }
        if(gamepad2.b && !bpressed) {
            bpressed = true;
            b = !b;
        }
        if(!gamepad2.b) {
            bpressed = false;
        }
        if(gamepad1.b && !b1pressed) {
            b1pressed = true;
            readying = true;
            ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(PP, PI, PD, PF);
            ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(PP, PI, PD, PF);
            start(robot, 14.8);
        }
        if(!gamepad1.b) {
            b1pressed = false;
        }
        if(gamepad2.dpad_right && !rightpressed) {
            rightpressed = true;
            firstshotvel = 16.0;
            secondshotvel = 16.0;
            thirdshotvel = 15.8;
            shots++;
            if(shots % 3 == 0) {
                //odometry.shootreset();
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
            //vel = (((maxvel - oldvel) / 4) * (odometry.getPoint().distance(target, Unit.FEET) - 10)) + maxvel;
            vel = firstshotvel;
            attempts = 0;
            shooting = true;
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.GOAL;
        }
        if(!gamepad2.dpad_right) {
            rightpressed = false;
        }
        if(gamepad2.right_trigger > 0.05 && !rt) {
            shots++;
            if(shots % 3 == 0) {
                //odometry.shootreset();
            }
            thing = oldthing;
            rt = true;
            /*
            Point target;
            if(handler.contains("Color") && handler.getData("Color").toString().equalsIgnoreCase("Blue")) {
                target = new Point(-3, 6);
            }
            else {
                target = new Point(3, 6);
            }
            //vel = (((maxvel - oldvel) / 4) * (odometry.getPoint().distance(target, Unit.FEET) - 10)) + maxvel;
             */
            vel = firstshotvel;
            attempts = 0;
            shooting = true;
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.GOAL;
        }
        if(gamepad2.x && !xpressed) {
            thing = 2;
            xpressed = true;
            if(readying) {
                readying = false;
            }
            else {
                readying = true;
                ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(PP, PI, PD, PF);
                ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(PP, PI, PD, PF);
                start(robot, 14.5);
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
            vel = firstshotvel;
            start(robot, vel);
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
        if(gamepad2.dpad_up && !dpadup) {
            dpadup = true;
            firstshotvel += 0.08;
            secondshotvel += 0.08;
            //thirdshotvel += 0.08;
        }
        if(!gamepad2.dpad_up) {
            dpadup = false;
        }
        if(gamepad2.dpad_down && !dpaddown) {
            dpaddown = true;
            firstshotvel -= 0.08;
            secondshotvel -= 0.08;
            //thirdshotvel -= 0.08;
        }
        if(!gamepad2.dpad_down) {
            dpaddown = false;
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
        if(robot.shoot1 != null && robot.shoot2 != null && shooting && (ready || (Math.abs(Math.abs(v) - Math.abs(t)) < 0.15) && Math.abs(a) < 0.5)) {
            if(handler.contains("stv") && Math.abs((double)handler.getData("stv")) > Math.min(Math.min(Math.abs(firstshotvel), Math.abs(secondshotvel)), Math.abs(thirdshotvel)) - 0.7) {
                ((DcMotorEx) robot.shoot1).setVelocityPIDFCoefficients(Shooter.p, 0, Shooter.d, Shooter.f);
                ((DcMotorEx) robot.shoot2).setVelocityPIDFCoefficients(Shooter.p, 0, Shooter.d, Shooter.f);
            }
            else {
                ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(PP, PI, PD, PF);
                ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(PP, PI, PD, PF);
            }
            ready = true;
            shoot(targ, robot);
        }
        else if(shooting && robot.shoot1 != null && robot.shoot2 != null) {
            handler.pushData("stv", vel);
            if(handler.contains("stv") && Math.abs((double)handler.getData("stv")) > Math.min(Math.min(Math.abs(firstshotvel), Math.abs(secondshotvel)), Math.abs(thirdshotvel)) - 0.7) {
                ((DcMotorEx) robot.shoot1).setVelocityPIDFCoefficients(Shooter.IP, Shooter.i, 0, 0);
                ((DcMotorEx) robot.shoot2).setVelocityPIDFCoefficients(Shooter.IP, Shooter.i, 0, 0);
            }
            else {
                ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(PP, PI, PD, PF);
                ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(PP, PI, PD, PF);
            }
        }
        else if(robot.shoot1 != null && robot.shoot2 != null) {
            if(handler.contains("stv") && Math.abs((double)handler.getData("stv")) > Math.min(Math.min(Math.abs(firstshotvel), Math.abs(secondshotvel)), Math.abs(thirdshotvel)) - 0.7) {
                ((DcMotorEx) robot.shoot1).setVelocityPIDFCoefficients(Shooter.IP, Shooter.i, 0, 0);
                ((DcMotorEx) robot.shoot2).setVelocityPIDFCoefficients(Shooter.IP, Shooter.i, 0, 0);
            }
            else {
                ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(PP, PI, PD, PF);
                ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(PP, PI, PD, PF);
            }
        }
        if(!shooting && !readying) {
            if(targ == Target.POWER) {
                handler.pushData("stv", powershotvel);
            }
            else {
                handler.pushData("stv", b ? 0 : firstshotvel);
            }
        }
        handler.pushData("gamepad2", g2);
    }

    public void shoot(GOFHardware robot, double velocity, boolean once) {
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
        if(((Math.abs(Math.abs(v) - Math.abs(t)) < 0.15) && Math.abs(a) < 0.25)) {
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
        if(targ == Target.POWER) {
            shootonce(target, robot);
            return;
        }
        if(step == 0 && robot.flicker != null) {
            attempts++;
            if (attempts == thing) {
                if(Math.abs(vel) < Math.abs(Math.min(Math.min(firstshotvel, secondshotvel), thirdshotvel)) - 0.7) {
                    uh = false;
                    powershots++;
                }
                else {
                    firstshotvel = oldfirstshotvel;
                    secondshotvel = oldsecondshotvel;
                    thirdshotvel = oldthirdshotvel;
                }
                shooting = false;
                if (target != Target.POWER) {
                    readying = false;
                }
                ready = false;
                ((DcMotorEx) robot.shoot1).setVelocityPIDFCoefficients(Shooter.p, Shooter.i, Shooter.d, 0);
                ((DcMotorEx) robot.shoot2).setVelocityPIDFCoefficients(Shooter.p, Shooter.i, Shooter.d, 0);
                return;
            }
            if (targ == Target.GOAL) {
                if (attempts == 1) {
                    vel = firstshotvel;
                    handler.pushData("stv", vel);
                } else if (attempts == 2) {
                    vel = secondshotvel;
                    handler.pushData("stv", vel);
                } else {
                    vel = thirdshotvel;
                    handler.pushData("stv", vel);
                }
            }
            //PIDFCoefficients pid = ((DcMotorEx)robot.shoot1).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            //((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(pid.p, pid.i, pid.d, 100);
            //((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(pid.p, pid.i, pid.d, 100);
            //timeofshot = System.currentTimeMillis();
            robot.flicker.setPosition(shootIn);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 1 && System.currentTimeMillis() - time > shootTime) {
            robot.flicker.setPosition(shootOut);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > shootTime * 1.6) {
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
            shooting = false;
            waitingForDrive = true;
            Drivetrain.waitingForShoot = false;
        }
    }
}
