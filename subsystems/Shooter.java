package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    boolean xpressed = false;
    boolean dpadup = false;
    boolean dpaddown = false;
    boolean ready = false;
    boolean b1pressed = false;

    public boolean shot = false;
    public boolean shooting = false;
    public boolean readying = false;

    public int powershots = 0;

    public Target targ;

    double time = 0;
    int step = 0;
    public int attempts = 0;

    public double v = 0;
    public double t = 0;
    public double a = 0;

    public double reallasttime = System.currentTimeMillis();
    public double lasttime = System.currentTimeMillis();
    public double lastv = 0;

    public static double shootTime = 75.0;
    public static double shootIn = 0.36;
    public static double shootOut = 0.55;

    public static double p = 0.2;
    public static double i = 0.2;
    public static double d = 0;
//    public static double f = 0;
//    public static double IP = 150;

//    public static double PP = 30;
//    public static double PI = 2;
//    public static double PD = 0;
//    public static double PF = 0;

    public static double vel = 16.22;
    public static double firstshotvel = 16.22;
    public static double secondshotvel = 16.22;
    public static double thirdshotvel = 16.22;

    //public static double oldfirstshotvel = 16.32;
    //public static double oldsecondshotvel = 16.32;
    //public static double oldthirdshotvel = 16.32;

    public static int thing = 4;
    public static int oldthing = 4;

    public static double powershotvel = 15.08;

    public double cycles = 0;

    public int shots = 0;

    public static boolean waitingForDrive = false;

    public double lasterror = 0;
    public double integral = 0;

    private Dictionary<Double, Double> integrals = new Hashtable<>();
    private Dictionary<Double, Double> powershotintegrals = new Hashtable<>();

    public Shooter(State state) {
        this.state = state;
    }

    public enum Target {
        GOAL,
        POWER
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, double angle, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
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
        if((gamepad1.b && !gamepad1.start) && !b1pressed) {
            b1pressed = true;
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
        if(!(gamepad1.b && !gamepad1.start)) {
            b1pressed = false;
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
        if(gamepad2.right_trigger < 0.05 && rt) {
            rt = false;
        }
        if((gamepad2.left_trigger > 0.05 && !lt) || Intake.rings == 3) {
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
        if(gamepad2.left_trigger < 0.05) {
            lt = false;
        }
        if(gamepad2.left_bumper) {
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
            Drivetrain.waitingForShoot = false;
            integral = 0;
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
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
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
                robot.shoot1.setPower(0);
                robot.shoot2.setPower(0);
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
                robot.shoot1.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
                robot.shoot2.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
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
        if(gamepad2.a && !apressed) {
            apressed = true;
            g2 = !g2;
        }
        if(!gamepad2.a) {
            apressed = false;
        }
        if (robot.shoot1 != null && robot.shoot2 != null && shooting && (ready || (((Math.abs(v) > powershotvel && /* NORMAL CONSTRAINT --> */ Math.abs(Math.abs(v) - Math.abs(t)) < 0.2) || /* POWER SHOT CONSTRAINTS --> */ (Math.abs(Math.abs(v) - Math.abs(t)) < 0.2 && Math.abs(a) < 2.0))))) {
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
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
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
                robot.shoot1.setPower(0);
                robot.shoot2.setPower(0);
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
                robot.shoot1.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
                robot.shoot2.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
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
        if(((Math.abs(Math.abs(v) - Math.abs(t)) < 0.2)) && Math.abs(a) < 2.0) {
            if(once) {
                shootonce(targ, robot);
            }
            else {
                shoot(targ, robot);
            }
        }
    }

    public void forceshoot(GOFHardware robot, double velocity, boolean once) {
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
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
                robot.shoot1.setPower(0);
                robot.shoot2.setPower(0);
                integral = 0;
            } else {
                integral += error * deltatime;
                if (v == powershotvel) {
                    powershotintegrals.put(Math.round(10 * v) / 10.0, integral);
                } else {
                    integrals.put(Math.round(10 * v) / 10.0, integral);
                }
                robot.shoot1.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
                robot.shoot2.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
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
        if(t > 16.8) {
            if (((Math.abs(Math.abs(v) - Math.abs(t)) < 0.25) && Math.abs(a) < 2.0)) {
                if (once) {
                    shootonce(targ, robot);
                } else {
                    shoot(targ, robot);
                }
            }
        }
        else {
            if (((Math.abs(Math.abs(v) - Math.abs(t)) < 0.2))) {
                if (once) {
                    shootonce(targ, robot);
                } else {
                    shoot(targ, robot);
                }
            }
        }
    }

    public void start(GOFHardware robot, double velocity) {
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
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
                robot.shoot1.setPower(0);
                robot.shoot2.setPower(0);
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
                robot.shoot1.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
                robot.shoot2.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
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
        robot.d1.setPosition(0);
        robot.d2.setPosition(0.39);
        if(targ == Target.POWER) {
            shootonce(targetlol, robot);
            return;
        }
        if(step == 0 && robot.flicker != null) {
            attempts++;
            if (attempts == thing) {
                shot = true;
                if(targetlol != Target.POWER) {
                    integral = 0;
                }
                if(Math.abs(vel) < Math.abs(Math.min(Math.min(firstshotvel, secondshotvel), thirdshotvel)) - 0.7 || Math.abs(vel) == powershotvel) {
                    uh = false;
                    powershots++;
                    if(powershots % 3 != 0) {
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
                }
                //else {
                //firstshotvel = oldfirstshotvel;
                //secondshotvel = oldsecondshotvel;
                //thirdshotvel = oldthirdshotvel;
                //}
                shooting = false;
                if (targetlol != Target.POWER) {
                    readying = false;
                }
                ready = false;
                robot.d1.setPosition(0);
                robot.d2.setPosition(0.39);
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
            time = System.currentTimeMillis();
            step++;
        }
        if(step == 2 && System.currentTimeMillis() - time > shootTime) {
            step = 0;
            shooting = false;
            if(powershots % 3 != 0) {
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
    }

    public void PIDReset() {
        integral = 0;
        lasttime = System.currentTimeMillis();
    }
}
