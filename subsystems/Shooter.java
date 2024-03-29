package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;

@Config
public class Shooter implements Subsystem {

    private State state;

    boolean apressed = false;
    boolean rightpressed = false;
    boolean rt = false;
    boolean lt = false;
    boolean g2 = true;
    boolean uh = false;
    boolean uh2 = false;
    boolean uh3 = false;
    boolean xpressed = false;
    boolean dpadup = false;
    boolean dpaddown = false;
    boolean ready = false;
    boolean b1pressed = false;
    boolean override = false;
    public static boolean off = true;

    public boolean shot = false;
    public boolean shooting = false;
    public boolean readying = false;

    public int powershots = 0;

    public Target targ;

    double time = 0;
    public int step = 0;
    public int attempts = 0;

    public double v = 0;
    public double t = 0;
    public double a = 0;

    public double reallasttime = System.currentTimeMillis();
    public double lasttime = System.currentTimeMillis();
    public double lastv = 0;

    public static double shootTime = 50.0;
    public static double shootIn = 0.38;
    public static double shootOut = 0.57;

    public static double p = 60;
    public static double i = 0.8;
    public static double d = 0.1;
    public static double f = 10;
    public static double IP = 150;

    public static double PP = 30;
    public static double PI = 2;
    public static double PD = 0;
    public static double PF = 0;

    public static double vel = 16.4;
    public static double firstshotvel = 16.4;
    public static double secondshotvel = 16.4;
    public static double thirdshotvel = 16.4;

    public static double shotvel = 0;

    public static int thing = 5;
    public static int oldthing = 5;

    public static double powershotvel = 15.2;

    public double cycles = 0;

    public int shots = 0;

    public static boolean waitingForDrive = false;

    public double lasterror = 0;
    public double integral = 0;

    public double endshootingtimer = System.currentTimeMillis();
    public double beginshootingtimer = System.currentTimeMillis();

    private Dictionary<Double, Double> integrals = new Hashtable<>();
    private Dictionary<Double, Double> powershotintegrals = new Hashtable<>();

    public Shooter(State state) {
        this.state = state;
    }

    public enum Target {
        HIGH,
        MID,
        LOW,
        POWER
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, double angle, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
        ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(p, i, d, f);
        ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(p, i, d, f);
        handler.pushData("Power Shots", powershots);
        if(!uh && (gamepad2.y || Drivetrain.waitingForShoot)) {
            uh = true;
            thing = 2;
            vel = powershotvel;
            attempts = 0;
            shooting = true;
            Intake.rings -= (thing - 1);
            if(Intake.rings < 0) {
                Intake.rings = 0;
            }
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.POWER;
            try {
                double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                if(target != 0) {
                    double closest = 0;
                    double closestdist = Double.MAX_VALUE;
                    Enumeration<Double> enumthing = powershotintegrals.keys();
                    while (enumthing.hasMoreElements()) {
                        double next = enumthing.nextElement();
                        if (Math.abs(next - target) < closestdist) {
                            closestdist = Math.abs(next - target);
                            closest = next;
                        }
                    }
                    integral = powershotintegrals.get(closest);
                }
            }
            catch(Exception e) {}
        }
        if(uh && !gamepad2.y) {
            uh = false;
        }
        if(!uh2 && ((gamepad2.left_trigger > 0.05))) {
            uh2 = true;
            thing = 5;
            vel = 13.8;
            attempts = 0;
            shooting = true;
            Intake.rings -= (thing - 1);
            if(Intake.rings < 0) {
                Intake.rings = 0;
            }
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.MID;
            try {
                double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                if(target != 0) {
                    double closest = 0;
                    double closestdist = Double.MAX_VALUE;
                    Enumeration<Double> enumthing = powershotintegrals.keys();
                    while (enumthing.hasMoreElements()) {
                        double next = enumthing.nextElement();
                        if (Math.abs(next - target) < closestdist) {
                            closestdist = Math.abs(next - target);
                            closest = next;
                        }
                    }
                    integral = powershotintegrals.get(closest);
                }
            }
            catch(Exception e) {}
        }
        if(uh3 && !gamepad2.x) {
            uh3 = false;
        }
        if(!uh3 && gamepad2.x) {
            uh3 = true;
            thing = 5;
            vel = 9.0;
            attempts = 0;
            shooting = true;
            Intake.rings -= (thing - 1);
            if(Intake.rings < 0) {
                Intake.rings = 0;
            }
            ready = false;
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.MID;
            try {
                double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                if(target != 0) {
                    double closest = 0;
                    double closestdist = Double.MAX_VALUE;
                    Enumeration<Double> enumthing = powershotintegrals.keys();
                    while (enumthing.hasMoreElements()) {
                        double next = enumthing.nextElement();
                        if (Math.abs(next - target) < closestdist) {
                            closestdist = Math.abs(next - target);
                            closest = next;
                        }
                    }
                    integral = powershotintegrals.get(closest);
                }
            }
            catch(Exception e) {}
        }
        if(uh2 && !(gamepad2.left_trigger > 0.05)) {
            uh2 = false;
        }
        if(((gamepad1.a || Drivetrain.bored) && !gamepad1.start) && !b1pressed) {
            b1pressed = true;
            readying = true;
            start(robot, powershotvel);
            try {
                double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                if(target != 0) {
                    double closest = 0;
                    double closestdist = Double.MAX_VALUE;
                    Enumeration<Double> enumthing = powershotintegrals.keys();
                    while (enumthing.hasMoreElements()) {
                        double next = enumthing.nextElement();
                        if (Math.abs(next - target) < closestdist) {
                            closestdist = Math.abs(next - target);
                            closest = next;
                        }
                    }
                    integral = powershotintegrals.get(closest);
                }
            }
            catch(Exception e) {}
        }
        if(!(gamepad1.b && !gamepad1.start)) {
            b1pressed = false;
        }
        if(gamepad2.right_trigger > 0.05 && !rt) {
            step = 0;
            shots++;
            if(shots % 3 == 0) {
                odometry.shootreset();
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
            vel = (((maxvel - oldvel) / 4) * (odometry.getPoint().distance(target, Unit.FEET) - 10)) + maxvel;
             */
            vel = firstshotvel;
            attempts = 0;
            shooting = true;
            Intake.rings = 0;
            ready = false;
            try {
                double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                if(target != 0) {
                    double closest = 0;
                    double closestdist = Double.MAX_VALUE;
                    Enumeration<Double> enumthing = integrals.keys();
                    while (enumthing.hasMoreElements()) {
                        double next = enumthing.nextElement();
                        if (Math.abs(next - target) < closestdist) {
                            closestdist = Math.abs(next - target);
                            closest = next;
                        }
                    }
                    integral = integrals.get(closest);
                }
            }
            catch(Exception e) {}
            handler.pushData("stv", vel);
            t = (double)handler.getData("stv");
            targ = Target.HIGH;
        }
        /*
        if(gamepad2.x && !xpressed) {
            thing = 2;
            xpressed = true;
            if(readying) {
                readying = false;
            }
            else {
                readying = true;
                start(robot, powershotvel);
                try {
                    double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                    if(target != 0) {
                        double closest = 0;
                        double closestdist = Double.MAX_VALUE;
                        Enumeration<Double> enumthing = integrals.keys();
                        while (enumthing.hasMoreElements()) {
                            double next = enumthing.nextElement();
                            if (Math.abs(next - target) < closestdist) {
                                closestdist = Math.abs(next - target);
                                closest = next;
                            }
                        }
                        integral = integrals.get(closest);
                    }
                }
                catch(Exception e) {}
            }
        }
        if(!gamepad2.x) {
            xpressed = false;
        }
         */
        if(gamepad2.right_trigger < 0.05 && rt) {
            rt = false;
        }
        if(Intake.rings == 3) {
            off = false;
            readying = true;
            vel = firstshotvel;
            start(robot, vel);
            try {
                double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                if(target != 0) {
                    double closest = 0;
                    double closestdist = Double.MAX_VALUE;
                    Enumeration<Double> enumthing = integrals.keys();
                    while (enumthing.hasMoreElements()) {
                        double next = enumthing.nextElement();
                        if (Math.abs(next - target) < closestdist) {
                            closestdist = Math.abs(next - target);
                            closest = next;
                        }
                    }
                    integral = integrals.get(closest);
                }
            }
            catch(Exception e) {}
        }
        if(gamepad2.left_bumper && !lt) {
            lt = true;
            if(off) {
                off = false;
                readying = true;
                vel = firstshotvel;
                start(robot, vel);
                try {
                    double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                    if(target != 0) {
                        double closest = 0;
                        double closestdist = Double.MAX_VALUE;
                        Enumeration<Double> enumthing = integrals.keys();
                        while (enumthing.hasMoreElements()) {
                            double next = enumthing.nextElement();
                            if (Math.abs(next - target) < closestdist) {
                                closestdist = Math.abs(next - target);
                                closest = next;
                            }
                        }
                        integral = integrals.get(closest);
                    }
                }
                catch(Exception e) {}
            }
            else {
                Intake.rings = 0;
                ((DcMotorEx)robot.shoot1).setVelocity(0);
                ((DcMotorEx)robot.shoot2).setVelocity(0);
                off = true;
                Drivetrain.waitingForShoot = false;
                integral = 0;
                shooting = false;
                ready = false;
                readying = false;
                robot.flicker.setPosition(shootOut);
            }
        }
        if(!gamepad2.left_bumper) {
            lt = false;
        }
        if(gamepad2.dpad_up && !dpadup) {
            dpadup = true;
            firstshotvel += 0.08;
            secondshotvel += 0.08;
            thirdshotvel += 0.08;
        }
        if(!gamepad2.dpad_up) {
            dpadup = false;
        }
        if(gamepad2.dpad_down && !dpaddown) {
            dpaddown = true;
            firstshotvel -= 0.08;
            secondshotvel -= 0.08;
            thirdshotvel -= 0.08;
        }
        if(!gamepad2.dpad_down) {
            dpaddown = false;
        }
        if(!handler.contains("stv")) {
            handler.pushData("stv", 0.0);
        }
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            if(deltatime > 0.2) {
                deltatime = 0;
            }
            double error = t - v;
            double derror = error - lasterror;
            if(derror >= error) {
                derror = 0;
            }
            double derivative = derror / deltatime;
            if(t == 0) {
                ((DcMotorEx)robot.shoot1).setVelocity(0);
                ((DcMotorEx)robot.shoot2).setVelocity(0);
                integral = 0;
            }
            else {
                integral += error * deltatime;
                if(v == powershotvel) {
                    powershotintegrals.put(Math.round(10 * v) / 10.0, integral);
                }
                else {
                    integrals.put(Math.round(10 * v) / 10.0, integral);
                }
                ((DcMotorEx)robot.shoot1).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
                ((DcMotorEx)robot.shoot2).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            }
            lasterror = error;
            double realdeltatime = (System.currentTimeMillis() - reallasttime) / 1000.0;
            if(realdeltatime > 0.2) {
                realdeltatime = 0;
                reallasttime = System.currentTimeMillis();
                lastv = v;
                a = 0.0;
            }
            if(v != lastv) {
                a = (v - lastv) / (realdeltatime);
                lastv = v;
                reallasttime = System.currentTimeMillis();
            }
            lasttime = System.currentTimeMillis();
            handler.pushData("sav", v);
            handler.pushData("saa", a);
        }
        if(gamepad2.a && !apressed) {
            apressed = true;
            g2 = !g2;
        }
        if(!gamepad2.a) {
            apressed = false;
        }
        if (robot.shoot1 != null && robot.shoot2 != null && shooting && ((ready && Math.abs((Math.abs(v) - Math.abs(t))) < 1.0) || (((Math.abs(v) > powershotvel && /* NORMAL CONSTRAINT --> */ (Math.abs(Math.abs(v) - Math.abs(t)) < 0.2)) || /* POWER SHOT CONSTRAINTS --> */ (Math.abs(v) <= powershotvel && Math.abs(Math.abs(v) - Math.abs(t)) < 0.35))))) {
            ready = true;
            shoot(targ, robot);
        } else if (shooting && robot.shoot1 != null && robot.shoot2 != null) {
            handler.pushData("stv", vel);
        }
        if(!shooting && !readying) {
            handler.pushData("stv", 0.0);
        }
        handler.pushData("gamepad2", g2);
    }

    public void shoot(GOFHardware robot, double velocity, boolean once) {
        ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(p, i, d, f);
        ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(p, i, d, f);
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            if(deltatime > 0.2) {
                deltatime = 0;
            }
            double error = t - v;
            double derror = error - lasterror;
            if(derror >= error) {
                derror = 0;
            }
            double derivative = derror / deltatime;
            if(t == 0) {
                ((DcMotorEx)robot.shoot1).setVelocity(0);
                ((DcMotorEx)robot.shoot2).setVelocity(0);
                integral = 0;
            }
            else {
                integral += error * deltatime;
                if(v == powershotvel) {
                    powershotintegrals.put(Math.round(10 * v) / 10.0, integral);
                }
                else {
                    integrals.put(Math.round(10 * v) / 10.0, integral);
                }
                ((DcMotorEx)robot.shoot1).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
                ((DcMotorEx)robot.shoot2).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            }
            lasterror = error;
            double realdeltatime = (System.currentTimeMillis() - reallasttime) / 1000.0;
            if(realdeltatime > 0.5) {
                realdeltatime = 0;
                reallasttime = System.currentTimeMillis();
                lastv = v;
                a = 0.0;
            }
            if(v != lastv) {
                a = (v - lastv) / (realdeltatime);
                lastv = v;
                reallasttime = System.currentTimeMillis();
            }
            lasttime = System.currentTimeMillis();
            handler.pushData("sav", v);
            handler.pushData("saa", a);
        }
        if(((Math.abs(Math.abs(v) - Math.abs(t)) < 0.2))) {
            if(once) {
                shootonce(targ, robot);
            }
            else {
                shoot(targ, robot);
            }
        }
    }

    public void forceshoot(GOFHardware robot, double velocity, boolean once) {
        ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(p, i, d, f);
        ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(p, i, d, f);
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            if(deltatime > 0.2) {
                deltatime = 0;
            }
            double error = t - v;
            double derror = error - lasterror;
            if (derror >= error) {
                derror = 0;
            }
            double derivative = derror / deltatime;
            if (t == 0) {
                ((DcMotorEx)robot.shoot1).setVelocity(0);
                ((DcMotorEx)robot.shoot2).setVelocity(0);
                integral = 0;
            } else {
                integral += error * deltatime;
                if (t == powershotvel) {
                    powershotintegrals.put(Math.round(10 * t) / 10.0, integral);
                } else {
                    integrals.put(Math.round(10 * v) / 10.0, integral);
                }
                ((DcMotorEx)robot.shoot1).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
                ((DcMotorEx)robot.shoot2).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            }
            lasterror = error;
            double realdeltatime = (System.currentTimeMillis() - reallasttime) / 1000.0;
            if(realdeltatime > 0.5) {
                realdeltatime = 0;
                reallasttime = System.currentTimeMillis();
                lastv = v;
                a = 0.0;
            }
            if(v != lastv) {
                a = (v - lastv) / (realdeltatime);
                lastv = v;
                reallasttime = System.currentTimeMillis();
            }
            lasttime = System.currentTimeMillis();
            handler.pushData("sav", v);
            handler.pushData("saa", a);
        }
        if ((override && ((velocity <= firstshotvel && Math.abs(Math.abs(v) - Math.abs(t)) < 0.3) || Math.abs(Math.abs(v) - Math.abs(t)) < 0.15)) || ((velocity <= firstshotvel && (Math.abs(Math.abs(v) - Math.abs(t)) < 0.2)) || (Math.abs(Math.abs(v) - Math.abs(t)) < 0.15))) {
            override = true;
            if (once) {
                shootonce(targ, robot);
            } else {
                shoot(targ, robot);
            }
        }
    }

    public void reallyforceshoot(GOFHardware robot, double velocity, boolean once) {
        ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(p, i, d, f);
        ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(p, i, d, f);
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            if(deltatime > 0.2) {
                deltatime = 0;
            }
            double error = t - v;
            double derror = error - lasterror;
            if (derror >= error) {
                derror = 0;
            }
            double derivative = derror / deltatime;
            if (t == 0) {
                ((DcMotorEx)robot.shoot1).setVelocity(0);
                ((DcMotorEx)robot.shoot2).setVelocity(0);
                integral = 0;
            } else {
                integral += error * deltatime;
                if (v == powershotvel) {
                    powershotintegrals.put(Math.round(10 * v) / 10.0, integral);
                } else {
                    integrals.put(Math.round(10 * v) / 10.0, integral);
                }
                ((DcMotorEx)robot.shoot1).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
                ((DcMotorEx)robot.shoot2).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            }
            lasterror = error;
            double realdeltatime = (System.currentTimeMillis() - reallasttime) / 1000.0;
            if(realdeltatime > 0.5) {
                realdeltatime = 0;
                reallasttime = System.currentTimeMillis();
                lastv = v;
                a = 0.0;
            }
            if(v != lastv) {
                a = (v - lastv) / (realdeltatime);
                lastv = v;
                reallasttime = System.currentTimeMillis();
            }
            lasttime = System.currentTimeMillis();
            handler.pushData("sav", v);
            handler.pushData("saa", a);
        }
        if (once) {
            shootonce(targ, robot);
        } else {
            shoot(targ, robot);
        }
    }

    public void start(GOFHardware robot, double velocity) {
        ((DcMotorEx)robot.shoot1).setVelocityPIDFCoefficients(p, i, d, f);
        ((DcMotorEx)robot.shoot2).setVelocityPIDFCoefficients(p, i, d, f);
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot2).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            if(deltatime > 0.2) {
                deltatime = 0;
            }
            double error = t - v;
            double derror = error - lasterror;
            if(derror >= error) {
                derror = 0;
            }
            double derivative = derror / deltatime;
            if(t == 0) {
                ((DcMotorEx)robot.shoot1).setVelocity(0);
                ((DcMotorEx)robot.shoot2).setVelocity(0);
                integral = 0;
            }
            else {
                integral += error * deltatime;
                if(t == powershotvel) {
                    powershotintegrals.put(Math.round(10 * t) / 10.0, integral);
                }
                else {
                    integrals.put(Math.round(10 * v) / 10.0, integral);
                }
                ((DcMotorEx)robot.shoot1).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
                ((DcMotorEx)robot.shoot2).setVelocity((((double)handler.getData("stv")) * 360.0) / (0.0254 * 4 * Math.PI * 99.5), AngleUnit.DEGREES);
            }
            lasterror = error;
            double realdeltatime = (System.currentTimeMillis() - reallasttime) / 1000.0;
            if(realdeltatime > 0.5) {
                realdeltatime = 0;
                reallasttime = System.currentTimeMillis();
                lastv = v;
                a = 0.0;
            }
            if(v != lastv) {
                a = (v - lastv) / (realdeltatime);
                lastv = v;
                reallasttime = System.currentTimeMillis();
            }
            lasttime = System.currentTimeMillis();
            handler.pushData("sav", v);
            handler.pushData("saa", a);
        }
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }

    public void shoot(Target targetlol, GOFHardware robot) {
        if(!Globals.AUTO) {
            robot.d1.setPosition(0);
            robot.d2.setPosition(0.8);
        }
        if(targ == Target.POWER) {
            shootonce(targetlol, robot);
            return;
        }
        if(step == 0 && robot.flicker != null) {
            attempts++;
            if(attempts == 1) {
                beginshootingtimer = System.currentTimeMillis();
            }
            if (attempts >= thing) {
                endshootingtimer = System.currentTimeMillis();
                shot = true;
                override = false;
                if(targetlol != Target.POWER) {
                    integral = 0;
                }
                shooting = false;
                if (targetlol != Target.POWER) {
                    readying = false;
                }
                ready = false;
                if(!Globals.AUTO) {
                    robot.d1.setPosition(Intake.height3.getX());
                    robot.d2.setPosition(Intake.height3.getY());
                    Intake.height = 3;
                }
                return;
            }
            if (!Globals.AUTO && targ == Target.HIGH) {
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
            robot.flicker.setPosition(shootIn);
            time = System.currentTimeMillis();
            shotvel = (double)handler.getData("sav");
            step++;
        }
        if(step == 1 && (System.currentTimeMillis() - time > shootTime)) {
            robot.flicker.setPosition(shootOut);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > shootTime * 1.6) {
            step = 0;
            shoot(targetlol, robot);
        }
    }

    public void shootonce(Target targetlol, GOFHardware robot) {
        if(step == 0 && robot.flicker != null) {
            robot.flicker.setPosition(shootIn);
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 1 && System.currentTimeMillis() - time > shootTime) {
            robot.flicker.setPosition(shootOut);
            shot = true;
            override = false;
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > shootTime) {
            step = 0;
            shooting = false;
            readying = true;
            start(robot, powershotvel);
            try {
                double target = (Math.round(10 * (((DcMotorEx) robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0) / 10.0);
                if(target != 0) {
                    double closest = 0;
                    double closestdist = Double.MAX_VALUE;
                    Enumeration<Double> enumthing = powershotintegrals.keys();
                    while (enumthing.hasMoreElements()) {
                        double next = enumthing.nextElement();
                        if (Math.abs(next - target) < closestdist) {
                            closestdist = Math.abs(next - target);
                            closest = next;
                        }
                    }
                    integral = powershotintegrals.get(closest);
                }
            }
            catch(Exception e) {}
        }
        powershots++;
        waitingForDrive = true;
        Drivetrain.waitingForShoot = false;
    }

    public void PIDReset() {
        integral = 0;
        lasttime = System.currentTimeMillis();
    }
}