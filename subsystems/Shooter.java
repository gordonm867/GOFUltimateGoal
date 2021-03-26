package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
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

    public double lasttime = System.currentTimeMillis();
    public double lastv = 0;

    public static double shootTime = 65.0;
    public static double shootIn = 0.36;
    public static double shootOut = 0.55;

    public static double p = 0.2;
    public static double i = 0.2;
    public static double d = 0;
    public static double f = 0;
    public static double IP = 150;

    public static double PP = 30;
    public static double PI = 2;
    public static double PD = 0;
    public static double PF = 0;

    public static double vel = 16.32;
    public static double firstshotvel = 16.32;
    public static double secondshotvel = 16.32;
    public static double thirdshotvel = 16.32;

    //public static double oldfirstshotvel = 16.32;
    //public static double oldsecondshotvel = 16.32;
    //public static double oldthirdshotvel = 16.32;

    public static int thing = 4;
    public static int oldthing = 4;

    public static double powershotvel = 14.9;

    public double cycles = 0;

    public int shots = 0;

    public static boolean waitingForDrive = false;

    public double lasterror = 0;
    public double integral = 0;

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
        if((gamepad1.b && !gamepad1.start) && !b1pressed) {
            b1pressed = true;
            readying = true;
            start(robot, powershotvel);
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
                start(robot, powershotvel);
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
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
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
            double error = t - v;
            double derror = error - lasterror;
            if(derror >= error) {
                derror = 0;
            }
            integral += error * deltatime;
            double derivative = derror / deltatime;
            if(t == 0) {
                robot.shoot1.setPower(0);
                robot.shoot2.setPower(0);
                integral = 0;
            }
            else {
                robot.shoot1.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
                robot.shoot2.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
            }
            lasterror = error;
            if(deltatime > 0.02) {
                a = (v - lastv) / (deltatime);
                lastv = v;
            }
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
        if(robot.shoot1 != null && robot.shoot2 != null && shooting && (ready || (Math.abs(Math.abs(v) - Math.abs(t)) < 0.15) && Math.abs(a) < 0.25)) {
            ready = true;
            shoot(targ, robot);
        }
        else if(shooting && robot.shoot1 != null && robot.shoot2 != null) {
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
            double error = t - v;
            double derror = error - lasterror;
            if(derror >= error) {
                derror = 0;
            }
            integral += error * deltatime;
            double derivative = derror / deltatime;
            if(t == 0) {
                robot.shoot1.setPower(0);
                robot.shoot2.setPower(0);
                integral = 0;
            }
            else {
                robot.shoot1.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
                robot.shoot2.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
            }
            lasterror = error;
            if(deltatime > 0.02) {
                a = (v - lastv) / (deltatime);
                lastv = v;
            }
            lasttime = System.currentTimeMillis();
            handler.pushData("sav", v);
        }
        if(((Math.abs(Math.abs(v) - Math.abs(t)) < 0.15))) {
            if(once) {
                shootonce(targ, robot);
            }
            else {
                shoot(targ, robot);
            }
        }
    }

    public void start(GOFHardware robot, double velocity) {
        handler.pushData("stv", velocity);
        if(robot.shoot1 != null && robot.shoot2 != null) {
            t = (double)handler.getData("stv");
            v = (((DcMotorEx)robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5) * 4 * Math.PI * 0.0254 / 360.0;
            double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
            double error = t - v;
            double derror = error - lasterror;
            if(derror >= error) {
                derror = 0;
            }
            integral += error * deltatime;
            double derivative = derror / deltatime;
            if(t == 0) {
                robot.shoot1.setPower(0);
                robot.shoot2.setPower(0);
                integral = 0;
            }
            else {
                robot.shoot1.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
                robot.shoot2.setPower(Range.clip(p * error + i * integral + d * derivative, -1, 1));
            }
            lasterror = error;
            if(deltatime > 0.02) {
                a = (v - lastv) / (deltatime);
                lastv = v;
            }
            lasttime = System.currentTimeMillis();
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
                integral = 0;
                if(Math.abs(vel) < Math.abs(Math.min(Math.min(firstshotvel, secondshotvel), thirdshotvel)) - 0.7) {
                    uh = false;
                    powershots++;
                }
                //else {
                    //firstshotvel = oldfirstshotvel;
                    //secondshotvel = oldsecondshotvel;
                    //thirdshotvel = oldthirdshotvel;
                //}
                shooting = false;
                if (target != Target.POWER) {
                    readying = false;
                }
                ready = false;
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
            integral = 0;
            shooting = false;
            waitingForDrive = true;
            Drivetrain.waitingForShoot = false;
        }
    }

    public void PIDReset() {
        integral = 0;
        lasttime = System.currentTimeMillis();
    }
}
