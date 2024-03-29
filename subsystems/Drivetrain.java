package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.vision.UGAngleHighGoalPipeline;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Config
public class Drivetrain implements Subsystem {

    private State state;
    private boolean field = false;
    private boolean changed = false;

    private boolean backwards = false;

    private double angleToHold = 0;
    public static double degRemaining = 40;
    //private double lasterror = 0;
    //private double lasttime = 0;

    //private double shooffset = 30;

    private boolean trigger;
    public boolean bpressed = false;
    public boolean xpressed = false;
    public boolean turningToPoint = false;
    public boolean turningToPoint2 = false;
    public boolean turningToPoint3 = false;

    public static boolean bored = false;

    private double error = 0;
    public double lasterror = 0;
    public double lasttime = 0;
    public double integral = 0;
    public double autoaimintegral = 0;
    public double autoaimlasttime = 0;
    public double lastyaw = 0;
    public double yaw = 0;

    public static double kp = 0.025;
    public static double kd = 0.003;
    public static double ki = 0.05;

    public static double autoaimkp = 0.025;
    public static double autoaimki = 0.05;
    public static double autoaimkd = 0.003;

    public static double mkp = 0.5;
    public static double mki = 0;
    public static double mkd = 0;
    public static double mks = 0.09;
    public static double mkv = 0.045;
    public static double mka = 0;
    public double mylasttime = 0;
    public double mylastv = 0;
    public double mintegral = 0;
    public Point lastpoint = null;

    public double lasttargetangle = Double.NaN;
    public double pstimer = 0;

    public static double minturn = 0.12;
    public double lastsign = 0;

    public double thisthing = 0;

    public static boolean waitingForShoot = false;

    public Powerstate powerstate = Powerstate.IDLE;
    public double targ = 0;

    public UGAngleHighGoalPipeline mypipeline = new UGAngleHighGoalPipeline(52);

    public Drivetrain(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public enum Powerstate {
        IDLE,
        FIRSTTRANSIT,
        WAIT,
        FIRST,
        SECONDTRANSIT,
        SECOND,
        THIRDTRANSIT,
        THIRD,
    }

    public void setpowerstate(Powerstate state) {
        this.powerstate = state;
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, double robotangle, RevBulkData data1, RevBulkData data2, Odometry odometry) {
        double drive = gamepad1.left_stick_y;
        double turn;
        if(Math.abs(gamepad1.right_stick_y) > 0.35) {
            turn = -gamepad1.right_stick_x * (1 - (gamepad1.right_stick_y * 0.5));
        }
        else {
            turn = -gamepad1.right_stick_x;
        }
        double angle = -gamepad1.left_stick_x;
        angle = Math.abs(angle) < 0.15 && Math.abs(drive) > 0.75 ? 0 : angle;
        drive = Math.abs(drive) < 0.15 && Math.abs(angle) > 0.75 ? 0 : drive;
        if(state == State.ON) {
            /* Precision vertical drive */
            if (gamepad1.dpad_down || gamepad1.dpad_up) {
                if (gamepad1.left_stick_y != 0) {
                    drive = drive * 0.1; // Slow down joystick driving
                } else {
                    if (gamepad1.dpad_up) {
                        drive = -0.1; // Slow drive forwards
                    } else {
                        drive = 0.1; // Slow drive backwards
                    }
                }
            }

            /* Precision sideways drive */
            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                if (gamepad1.right_stick_x != 0) {
                    angle = angle * 0.15; // Slow down joystick side movement
                } else {
                    if (gamepad1.dpad_left) {
                        angle = 0.15; // Slow leftwards
                    } else {
                        angle = -0.15; // Slow rightwards
                    }
                }
            }

            /* Precision turn */
            if (gamepad1.left_bumper) {
                turn = 0.1; // Slow left turn
            }
            if (gamepad1.right_bumper) {
                turn = -0.1; // Slow right turn
            }

            if(turn != 0 || drive != 0 || angle != 0) {
                turningToPoint = false;
                turningToPoint2 = false;
                turningToPoint3 = false;
                powerstate = Powerstate.IDLE;
                waitingForShoot = false;
            }
            if(gamepad2.left_bumper) {
                powerstate = Powerstate.IDLE;
                turningToPoint3 = false;
            }
            if(turningToPoint) {
                double old = Globals.MIN_SPEED;
                Globals.MIN_SPEED = minturn;
                double targetangle;
                if(handler.contains("Color") && handler.getData("Color").toString().equalsIgnoreCase("Blue")) {
                    targetangle = odometry.getPoint().angle(new Point(-3 + ((odometry.getX() + 3) * (7.5 / 12.0) / 5), 6), AngleUnit.DEGREES) - 1.5;
                }
                else {
                    targetangle = odometry.getPoint().angle(new Point(3 + ((odometry.getX() - 3) * (7.5 / 12.0) / 5), 6), AngleUnit.DEGREES) - 1.5;
                }
                /*
                if(Math.abs(Functions.normalize(robotangle - targetangle)) < 3 && handler.contains("Omega") && Math.abs((double)handler.getData("Omega")) > 20 && Math.signum((double)handler.getData("Omega")) == Math.signum(Functions.normalize(targetangle - robotangle))) {
                    //double[] pows = calcupdate(robot, odometry.getPoint(), odometry, targetangle, robotangle, data1);
                    //robot.setDrivePower(Math.signum(-pows[0]), Math.signum(-pows[1]), Math.signum(-pows[2]), Math.signum(-pows[3]));
                    robot.setDrivePower(0, 0, 0, 0);
                }
                 */
                if(Math.abs(Functions.normalize(robotangle - targetangle)) < 0.5 && ((!handler.contains("Omega")) || (handler.contains("Omega") && Math.abs((double)handler.getData("Omega")) < 20)))  {
                    robot.setDrivePower(0, 0, 0, 0);
                    //turningToPoint = false;
                    return;
                }
                if(Math.signum(Functions.normalize(robotangle - targetangle)) == -lastsign) {
                    minturn -= 0.01;
                }
                else if(Math.abs(Functions.normalize(robotangle - targetangle)) > 0.5 && handler.contains("Omega") && Math.abs((double)handler.getData("Omega")) < 0.1) {
                    minturn += 0.01;
                }
                update(robot, odometry.getPoint(), odometry, targetangle, robotangle, data1);
                Globals.MIN_SPEED = old;
                lastsign = Math.signum(Functions.normalize(robotangle - targetangle));
                return;
            }
            if(turningToPoint2) {
                lastyaw = yaw;
                boolean red = (!handler.contains("Color") || !((String) handler.getData("Color")).equalsIgnoreCase("Blue"));
                yaw = mypipeline.calculateYaw(red ? UGAngleHighGoalPipeline.Target.RED : UGAngleHighGoalPipeline.Target.BLUE);
                double error = Functions.normalize((red ? -7.8 : -10.5) - yaw);
                double dtime = (System.currentTimeMillis() - autoaimlasttime) / 1000.0;
                double derivative = (yaw-lastyaw) / dtime;
                autoaimintegral += error * dtime;
                autoaimlasttime = System.currentTimeMillis();
                double out = (autoaimkp * error) + (autoaimki * autoaimintegral) + (autoaimkd * derivative);
                if(Math.abs(error) > 0.2 && yaw != 0) {
                    robot.setDrivePower(out, out, -out, -out);
                    return;
                }
                else {
                    turningToPoint2 = false;
                }
            }
            Globals.MAX_SPEED = 1.0;
            //Globals.MIN_SPEED = 0.25;
            handler.pushData("Power State", powerstate);
            double displacement;
            if(turningToPoint3) {
                if(powerstate == Powerstate.FIRSTTRANSIT) {
                    waitingForShoot = false;
                    if (handler.contains("Color") && handler.getData("Color").toString().equalsIgnoreCase("Blue")) {
                        Globals.MIN_SPEED = 0.3;
                        if (Math.abs(robotangle - 90) > 0.5) {
                            dupdate(robot, 90, robotangle);
                        }
                        else {
                            robot.setDrivePower(0, 0, 0, 0);
                            powerstate = Powerstate.FIRST;
                        }
                    }
                    else {
                        Globals.MIN_SPEED = 0.3;
                        if (Math.abs(robotangle - 90) > 0.5) {
                            dupdate(robot, 90, robotangle);
                        }
                        else {
                            robot.setDrivePower(0, 0, 0, 0);
                            powerstate = Powerstate.FIRST;
                            Shooter.waitingForDrive = false;
                            Shooter.powershotvel += 0.1;
                        }
                    }
                    return;
                }
                else if(powerstate == Powerstate.FIRST) {
                    robot.setDrivePower(0, 0, 0, 0);
                    if(Shooter.waitingForDrive) {
                        Shooter.waitingForDrive = false;
                        waitingForShoot = false;
                        powerstate = Powerstate.SECONDTRANSIT;
                        Shooter.powershotvel -= 0.1;
                        pstimer = System.currentTimeMillis();
                    }
                    else if(odometry.getVelocity() * 1000 <= 0.1) {
                        waitingForShoot = true;
                    }
                    return;
                }
                else if(powerstate == Powerstate.SECONDTRANSIT) {
                    if(System.currentTimeMillis() - pstimer >= 750) {
                        waitingForShoot = false;
                        if (handler.contains("Color") && handler.getData("Color").toString().equalsIgnoreCase("Blue")) {
                            Globals.MIN_SPEED = 0.3;
                            if (Math.abs(robotangle - 95.5) > 0.5) {
                                dupdate(robot, 95.5, robotangle);
                            }
                            else {
                                robot.setDrivePower(0, 0, 0, 0);
                                powerstate = Powerstate.SECOND;
                            }
                        }
                        else {
                            Globals.MIN_SPEED = 0.3;
                            if (Math.abs(robotangle - 95.5) > 0.5) {
                                dupdate(robot, 95.5, robotangle);
                            }
                            else {
                                robot.setDrivePower(0, 0, 0, 0);
                                powerstate = Powerstate.SECOND;
                                Shooter.waitingForDrive = false;
                            }
                        }
                    }
                    else {
                        robot.setDrivePower(0, 0, 0, 0);
                    }
                    return;
                }
                else if(powerstate == Powerstate.SECOND) {
                    robot.setDrivePower(0, 0, 0, 0);
                    if(Shooter.waitingForDrive) {
                        Shooter.waitingForDrive = false;
                        waitingForShoot = false;
                        powerstate = Powerstate.THIRDTRANSIT;
                        pstimer = System.currentTimeMillis();
                    }
                    else if(odometry.getVelocity() * 1000 <= 0.1) {
                        waitingForShoot = true;
                    }
                    return;
                }
                else if(powerstate == Powerstate.THIRDTRANSIT) {
                    if(System.currentTimeMillis() - pstimer >= 750) {
                        waitingForShoot = false;
                        if (handler.contains("Color") && handler.getData("Color").toString().equalsIgnoreCase("Blue")) {
                            Globals.MIN_SPEED = 0.3;
                            if (Math.abs(angle - 103.5) > 0.5) {
                                dupdate(robot, 103.5, robotangle);
                            } else {
                                robot.setDrivePower(0, 0, 0, 0);
                                powerstate = Powerstate.THIRD;
                            }
                        } else {
                            Globals.MIN_SPEED = 0.3;
                            if (Math.abs(robotangle - 103.5) > 0.5) {
                                dupdate(robot, 103.5, robotangle);
                            } else {
                                robot.setDrivePower(0, 0, 0, 0);
                                powerstate = Powerstate.THIRD;
                                Shooter.waitingForDrive = false;
                            }
                        }
                    }
                    else {
                        robot.setDrivePower(0, 0, 0, 0);
                    }
                    return;
                }
                else if(powerstate == Powerstate.THIRD) {
                    robot.setDrivePower(0, 0, 0, 0);
                    if(Shooter.waitingForDrive) {
                        turningToPoint3 = false;
                        powerstate = Powerstate.IDLE;
                        waitingForShoot = false;
                        Shooter.waitingForDrive = false;
                    }
                    else if(odometry.getVelocity() * 1000 <= 0.1) {
                        waitingForShoot = true;
                    }
                    return;
                }
                return;
            }
            if(gamepad1.start && gamepad1.y && !changed) {
                field = !field;
                changed = true;
            }
            if(changed && !(gamepad1.start && gamepad1.y)) {
                changed = false;
            }
            if(!handler.contains("Angle")) {
                handler.pushData("Angle", robotangle);
            }
            if(turn != 0 || (drive == 0 && angle == 0)) {
                angleToHold = (double)handler.getData("Angle");
            }
            if(!field) {
                drive = adjust(drive);
                turn = adjust(turn);
                angle = adjust(angle);
                double scaleFactor;
                //double maxspeed = Math.min(Globals.MAX_SPEED, Math.max(Math.abs(robot.lf.getPower()), Math.max(Math.abs(robot.lb.getPower()), Math.max(Math.abs(robot.rf.getPower()), Math.abs(robot.rb.getPower())))) + 0.2);
                double max = Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)));
                if (max > Globals.MAX_SPEED) {
                    scaleFactor = Globals.MAX_SPEED / max;
                } else {
                    scaleFactor = Globals.MAX_SPEED;
                }
                scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
                error = Functions.normalize(angleToHold - (double)handler.getData("Angle"));
                //turn += (kp * error * Globals.MAX_SPEED);
                robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
            }
            else {
                double maxspeed = Math.max(Math.abs(drive + angle), Math.abs(drive - angle));
                double driveangle = Math.atan2(drive, angle);
                double relAngle = Math.toRadians(Math.toDegrees(driveangle) - ((double)handler.getData("Angle")));
                drive = Math.cos(relAngle);
                angle = Math.sin(relAngle);
                double scaleFactor = maxspeed / Math.max(Math.abs(drive + angle), Math.abs(drive - angle));
                scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
                error = Functions.normalize(angleToHold - (double)handler.getData("Angle"));
                turn += (kp * error * Globals.MAX_SPEED);
                robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
            }
            if (gamepad1.b && !gamepad1.start && !xpressed) {
                xpressed = true;
                turningToPoint = true;
            }
            if(!gamepad1.b && !gamepad1.start) {
                xpressed = false;
            }

            if(gamepad1.left_trigger > 0.1 && !trigger) {
                trigger = true;
                turningToPoint2 = true;
                autoaimintegral = 0;
                autoaimlasttime = System.currentTimeMillis();
                yaw = mypipeline.calculateYaw((handler.contains("Color") && ((String)handler.getData("Color")).equalsIgnoreCase("Blue") ? UGAngleHighGoalPipeline.Target.BLUE : UGAngleHighGoalPipeline.Target.RED));
            }
            if(!(gamepad1.left_trigger > 0.1)) {
                trigger = false;
            }
            if((gamepad1.a || bored) && !gamepad1.start && !bpressed) {
                odometry.doresetthing();
                bored = false;
                bpressed = true;
                turningToPoint3 = true;
                integral = 0;
                lasttime = System.currentTimeMillis();
                lasterror = 0;
                powerstate = Powerstate.FIRSTTRANSIT;
            }
            if(!gamepad1.a) {
                bpressed = false;
            }
        }
        else if (state == State.OFF) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(double lr, double lf, double rr, double rf, GOFHardware robot) {
        if(state == State.ON) {
            robot.setDrivePower(lr, lf, rr, rf); // Set motors to values based on gamepad
        }
        else if(state == State.OFF) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(ArrayList<Double> powers, GOFHardware robot) {
        if(state == State.ON) {
            robot.setDrivePower(powers.get(0), powers.get(1), powers.get(2), powers.get(3)); // Set motors to values based on gamepad
        }
        else if(state == State.OFF) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    public void update(double[] powers, GOFHardware robot) {
        if(state == State.ON) {
            robot.setDrivePower(powers[0], powers[1], powers[2], powers[3]); // Set motors to values based on gamepad
        }
        else if(state == State.OFF) {
            robot.setDrivePower(0, 0, 0, 0);
        }
    }

    /**
     * Move towards point of specified displacement with smooth deceleration
     * @param robot Hardware instance
     * @param myAngle Target angle
     * @param current Current angle (we don't want to read again)
     */
    public void dupdate(GOFHardware robot, double myAngle, double current) {
        if(Math.abs(integral) > 1 / ki) {
            integral = 1/ki * Math.signum(integral);
        }
        if(Double.isNaN(myAngle) || Double.isNaN(lasttargetangle) || lasttargetangle != myAngle) {
            integral = 0;
            lasttime = System.currentTimeMillis();
            lasttargetangle = myAngle;
        }
        double turn = 0;
        if(!Double.isNaN(myAngle)) {
            error = Functions.normalize(myAngle - current);
            if(Math.abs(error) >= 0.1) {
                double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
                double deriv = (error - lasterror) / deltatime;
                if(Math.abs(error) < 1 / kp) {
                    integral += error * deltatime;
                }
                lasttime = System.currentTimeMillis();
                lasterror = error;
                turn = (kp * error) + (kd * deriv) + (ki * integral);
            }
            else {
                integral = 0;
            }
        }
        robot.setDrivePower(turn, turn, -turn, -turn);
    }

    /**
     * Move towards point with smooth deceleration
     * @param robot Hardware instance
     * @param target Our target point on the field
     * @param odometry Odometer
     * @param myAngle Target angle
     * @param current Current angle (we don't want to read again)
     * @param data Bulk data from REV hub
     */
    public void update(GOFHardware robot, Point target, Odometry odometry, double myAngle, double current, RevBulkData data) {
        if(Math.abs(integral) > 1 / ki) {
            integral = 1/ki * Math.signum(integral);
        }
        if(Double.isNaN(myAngle) || Double.isNaN(lasttargetangle) || lasttargetangle != myAngle) {
            integral = 0;
            lasttime = System.currentTimeMillis();
            lasttargetangle = myAngle;
        }
        if(lastpoint == null || !lastpoint.equals(target)) {
            mintegral = 0;
            mylasttime = System.currentTimeMillis();
            mylastv = odometry.getVelocity() * 1000.0;
        }
        lastpoint = target;
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            if(PIDd != -displacement) {
                angle = Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
                drive = PIDd;
                if(!Double.isNaN(myAngle)) {
                    error = Functions.normalize(myAngle - current);
                    if(Math.abs(error) >= 0.25) {
                        double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
                        double deriv = (error - lasterror) / deltatime;
                        if(Math.abs(error) < 1 / kp) {
                            integral += error * deltatime;
                        }
                        lasttime = System.currentTimeMillis();
                        lasterror = error;
                        turn = (kp * error) + (kd * deriv) + (ki * integral);
                    }
                    else {
                        integral = 0;
                    }
                }
            }
        }
        else if(!Double.isNaN(myAngle)) {
            error = Functions.normalize(myAngle - current);
            if(Math.abs(error) >= 0.4) {
                double deltatime = (System.currentTimeMillis() - lasttime) / 1000.0;
                double deriv = (error - lasterror) / deltatime;
                if(Math.abs(error) < 1 / kp) {
                    integral += error * deltatime;
                }
                lasttime = System.currentTimeMillis();
                lasterror = error;
                turn = (kp * error) + (kd * deriv) + (ki * integral);
            }
            else {
                integral = 0;
            }
        }
        double scaleFactor;
        double max = Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        double dT = (System.currentTimeMillis() - mylasttime) / 1000.0;
        double dV = (mylastv - (odometry.getVelocity() * 1000.0));
        double mderivative = dV / dT;
        mintegral += displacement * dT;
        if(Math.abs(displacement) < (1.0/96.0)) {
            mintegral = 0;
        }
        scaleFactor = (mkp * displacement) + (mki * mintegral) + (mkd * mderivative);
        targ = Math.sqrt(Math.pow((Globals.MAX_STRAIGHT * (drive / (Math.abs(drive) + Math.abs(angle)))), 2) + Math.pow((Globals.MAX_STRAFE * (angle / (Math.abs(angle) + Math.abs(drive)))), 2));
        double targa = 0;
        if(targ - (odometry.getVelocity() * 1000) > 0.5) {
            targa = Globals.MAX_ACCEL;
        }
        else if(targ - (odometry.getVelocity() * 1000) < 0.5) {
            targa = -Globals.MAX_ACCEL;
        }
        double feedforward = (mks * (12.0 / robot.battery.getVoltage())) + (mkv * (targ - (odometry.getVelocity() * 1000))) + (mka * (targa - mderivative));
        scaleFactor += feedforward;
        if(Math.abs(scaleFactor) > Globals.MAX_SPEED) {
            scaleFactor = Math.signum(scaleFactor) * Globals.MAX_SPEED;
        }
        double variablething = Globals.MIN_SPEED - (Math.abs(drive) + Math.abs(angle));
        if(Math.abs(turn) > 0 && Math.abs(max) < Globals.MIN_SPEED && variablething > 0) {
            turn = Math.signum(turn) * variablething;
            max = Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        }
        scaleFactor /= max;
        mylasttime = System.currentTimeMillis();
        mylastv = odometry.getVelocity() * 1000.0;
        odometry.update(data, current);
        if(Double.isNaN(scaleFactor)) {
            return;
        }
        robot.setDrivePower((scaleFactor * (drive - angle)) + turn, (scaleFactor * (drive + angle)) + turn, (scaleFactor * (drive + angle)) - turn, (scaleFactor * (drive - angle)) - turn);
    }

    /**
     * Move towards point with forceful deceleration
     * @param robot Hardware instance
     * @param target Our target point on the field
     * @param odometry Odometer
     * @param myAngle Target angle
     * @param velocity Current velocity
     * @param delta Change in displacement since last update
     * @param current Current angle (we don't want to read again)
     * @param data Bulk data from REV hub
     */
    public void update(GOFHardware robot, Point target, Odometry odometry, double myAngle, double velocity, double delta, double current, RevBulkData data) {
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            if(PIDd != -displacement) {
                angle = (1f / 0.8f) * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
                drive = PIDd;
                if(!Double.isNaN(myAngle)) {
                    error = Functions.normalize(myAngle - current);
                    if(Math.abs(error) >= 1.0) {
                        error = Functions.normalize(myAngle - current);
                        if (Math.abs(error + 360) < Math.abs(error)) {
                            error += 360;
                        }
                        if (Math.abs(error - 360) < Math.abs(error)) {
                            error -= 360;
                        }
                        double deriv = (error - lasterror) / (System.currentTimeMillis() - lasttime);
                        lasttime = System.currentTimeMillis();
                        lasterror = error;
                        double pow = (kp * error * Globals.MAX_SPEED) + (kd * deriv * Globals.MAX_SPEED);
                        turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                    }
                }
                if(Math.abs(displacement) <= (1.0/96.0) || (Math.abs(angle) < 0.00001 && Math.abs(drive) < 0.00001)) {
                    drive = 0;
                    angle = 0;
                    if(!Double.isNaN(myAngle)) {
                        error = Functions.normalize(myAngle - current);
                        if(Math.abs(error) >= 1.0) {
                            error = Functions.normalize(myAngle - current);
                            if (Math.abs(error + 360) < Math.abs(error)) {
                                error += 360;
                            }
                            if (Math.abs(error - 360) < Math.abs(error)) {
                                error -= 360;
                            }
                            double deriv = (error - lasterror) / (System.currentTimeMillis() - lasttime);
                            lasttime = System.currentTimeMillis();
                            lasterror = error;
                            double pow = (kp * error * Globals.MAX_SPEED) + (kd * deriv * Globals.MAX_SPEED);
                            turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                        }
                    }
                }
            }
        }
        double scaleFactor;
        double max = Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        if(max > 1) { // If our highest speed is greater than one
            backwards = false; // The robot is moving forward
            scaleFactor = Math.abs(Globals.MAX_SPEED / max); // Scale speed to max at our maximum speed parameter
        } else {
            if(displacement >= 0.5) { // If we are not within six inches of our target
                backwards = false; // The robot should move forward
                scaleFactor = Math.abs(Math.max(displacement * 2, Globals.MIN_SPEED) / max); // Scale speed to equal double our displacement from our target in feet
            }
            else { // If we have to decelerate
                if(delta < 0 || Math.abs(velocity) < 1) { // If our distance to our target increases or our velocity is sufficiently slow
                    backwards = false; // The robot should move forward
                    scaleFactor = Math.abs((Globals.MIN_SPEED / max)); // Move robot at minimum non-stall speed
                }
                else { // If our distance to our target is still decreasing and our velocity is still too high
                    backwards = true;
                    scaleFactor = Range.clip(((-1.25 * velocity) + 0.25), -Globals.MAX_SPEED, Globals.MIN_SPEED) / max; // Force the wheels backwards based on the current robot velocity
                }
            }
            if(Math.abs(scaleFactor) < Globals.MIN_SPEED) { // If the scale factor is particularly low
                scaleFactor = Globals.MIN_SPEED * Math.signum(scaleFactor); // Move it higher to ensure that we will not stall the robot
            }
        }
        odometry.update(data, current); // Update the odometry pose estimate
        robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Power the motors
    }

    /**
     * I genuinely have no clue if this is different than the method above
     * @param robot Hardware instance
     * @param target Our target point on the field
     * @param odometry Odometer
     * @param myAngle Target angle
     * @param velocity Current velocity
     * @param delta Change in displacement since last update
     * @param current Current angle (we don't want to read again)
     * @param data Bulk data from REV hub
     */
    public void clupdate(GOFHardware robot, Point target, Odometry odometry, double myAngle, double velocity, double delta, double current, RevBulkData data, RevBulkData data2) {
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            angle = (1f / 0.95f) * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            drive = PIDd;
            /*
            if(displacement > 1) {
                double firstAngle = myPos.angle(target, AngleUnit.DEGREES);
                double firsterror = Math.abs(current - firstAngle);
                double secondAngle = Functions.normalize(myPos.angle(target, AngleUnit.DEGREES) + 180);
                double seconderror = Math.abs(current - secondAngle);
                if(firsterror <= seconderror) {
                    myAngle = firstAngle;
                }
                else {
                    myAngle = secondAngle;
                }
            }
             */
            if (!Double.isNaN(myAngle)) {
                error = Functions.normalize(myAngle - current);
                if (Math.abs(error) >= 1.0) {
                    error = Functions.normalize(myAngle - current);
                    if (Math.abs(error + 360) < Math.abs(error)) {
                        error += 360;
                    }
                    if (Math.abs(error - 360) < Math.abs(error)) {
                        error -= 360;
                    }
                    double deriv = (error - lasterror) / (System.currentTimeMillis() - lasttime);
                    lasttime = System.currentTimeMillis();
                    lasterror = error;
                    double pow = (kp * error * Globals.MAX_SPEED) + (kd * deriv * Globals.MAX_SPEED);
                    turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                }
            }
        }
        if(Math.abs(displacement) <= (Math.sqrt(2) / 100) || (Math.abs(angle) < 0.00001 && Math.abs(drive) < 0.00001)) {
            drive = 0;
            angle = 0;
            if (!Double.isNaN(myAngle)) {
                error = Functions.normalize(myAngle - current);
                if (Math.abs(error) >= 1.0) {
                    error = Functions.normalize(myAngle - current);
                    if (Math.abs(error + 360) < Math.abs(error)) {
                        error += 360;
                    }
                    if (Math.abs(error - 360) < Math.abs(error)) {
                        error -= 360;
                    }
                    double deriv = (error - lasterror) / (System.currentTimeMillis() - lasttime);
                    lasttime = System.currentTimeMillis();
                    lasterror = error;
                    double pow = (kp * error * Globals.MAX_SPEED) + (kd * deriv * Globals.MAX_SPEED);
                    turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                }
            }
        }
        double scaleFactor;
        double max = Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        if(displacement >= 0.5 && max > 1) {
            backwards = false;
            scaleFactor = Math.abs(Globals.MAX_SPEED / max);
        } else {
            if(displacement >= 0.5) {
                backwards = false;
                scaleFactor = Math.abs((Math.max(Math.min(Globals.MAX_SPEED, displacement * 2), Globals.MIN_SPEED)) / max);
            }
            else {
                if(delta < 0 || Math.abs(velocity) < 1) {
                    backwards = false;
                    scaleFactor = Math.min(Globals.MAX_SPEED / max, Math.abs((Globals.MIN_SPEED / max)));
                }
                else {
                    backwards = true;
                    scaleFactor = Range.clip(((-1.25 * velocity) + 0.25), -Globals.MAX_SPEED, Globals.MIN_SPEED) / max;
                }
            }
            if(Math.abs(scaleFactor) < Globals.MIN_SPEED) {
                scaleFactor = Globals.MIN_SPEED * Math.signum(scaleFactor);
            }
        }
        /*
        telemetry.addData("Backwards", backwards);
        telemetry.update();
         */
        odometry.update(data, current);
        robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));
    }

    public double[] calcupdate(GOFHardware robot, Point target, Odometry odometry, double myAngle, double current, RevBulkData data) {
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            if(PIDd != -displacement) {
                angle = (1f / 0.95f) * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
                drive = PIDd;
                if(!Double.isNaN(myAngle)) {
                    error = Functions.normalize(myAngle - current);
                    if(Math.abs(error) >= 0.8) {
                        double deriv = (error - lasterror) / (System.currentTimeMillis() - lasttime);
                        lasttime = System.currentTimeMillis();
                        lasterror = error;
                        double pow = (kp * error * Globals.MAX_SPEED) + (kd * deriv * Globals.MAX_SPEED);
                        turn = Math.max(Math.abs(pow), (Globals.MIN_SPEED * (Math.abs(drive) + Math.abs(angle)) / Globals.MAX_SPEED)) * Math.signum(pow);
                    }
                }
                if(Math.abs(displacement) <= 1.0/96.0) {
                    drive = 0;
                    angle = 0;
                }
            }
        }
        else if(!Double.isNaN(myAngle)) {
            error = Functions.normalize(myAngle - current);
            if(Math.abs(error) >= 0.8) {
                double deriv = (error - lasterror) / (System.currentTimeMillis() - lasttime);
                lasttime = System.currentTimeMillis();
                lasterror = error;
                double pow = (kp * error * Globals.MAX_SPEED) + (kd * deriv * Globals.MAX_SPEED);
                turn = Math.max(Math.abs(pow), (Globals.MIN_SPEED * (Math.abs(drive) + Math.abs(angle)) / Globals.MAX_SPEED)) * Math.signum(pow);
            }
        }
        double scaleFactor;
        double max = Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        if(max > 1) {
            scaleFactor = Math.abs(Globals.MAX_SPEED / max);
        } else {
            if(Math.abs(current - myAngle) > 45 && displacement <= 0.5) {
                scaleFactor = Math.abs(Globals.MAX_SPEED / max);
            }
            else if(displacement >= 0.5) {
                scaleFactor = Math.abs(Math.max(displacement * 2, Globals.MIN_SPEED) / max);
            }
            else {
                scaleFactor = Math.abs(Math.max(Math.max(displacement / 2.5, Math.abs(Functions.normalize(myAngle - current)) / 90.0), Globals.MIN_SPEED) / max);
            }
        }
        return new double[] {scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)};
    }

    /**
     * Move towards point with no deceleration
     * @param robot Hardware instance
     * @param target Our target point on the field
     * @param odometry Odometer
     * @param myAngle Target angle
     * @param current Current angle (we don't want to read again)
     * @param data Bulk data from REV hub
     */
    public void fastupdate(GOFHardware robot, Point target, Odometry odometry, double myAngle, double current, RevBulkData data, RevBulkData data2) {
        Point myPos = odometry.getPoint();
        double displacement = Math.abs(Math.sqrt(Math.pow(target.getX() - myPos.getX(), 2) + Math.pow(target.getY() - myPos.getY(), 2)));
        double angle = 0;
        double drive = 0;
        double turn = 0;
        if(displacement != 0 && !Double.isInfinite(displacement) && !Double.isNaN(displacement)) {
            double PIDd = -Math.cos(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
            if(PIDd != -displacement) {
                angle = (1f / 0.95f) * Math.sin(myPos.angle(target, AngleUnit.RADIANS) - Math.toRadians(current)) * displacement;
                drive = PIDd;
                if(!Double.isNaN(myAngle)) {
                    error = Functions.normalize(myAngle - current);
                    if(Math.abs(error) >= 1.0) {
                        error = Functions.normalize(myAngle - current);
                        if (Math.abs(error + 360) < Math.abs(error)) {
                            error += 360;
                        }
                        if (Math.abs(error - 360) < Math.abs(error)) {
                            error -= 360;
                        }
                        double deriv = (error - lasterror) / (System.currentTimeMillis() - lasttime);
                        lasttime = System.currentTimeMillis();
                        lasterror = error;
                        double pow = (kp * error * Globals.MAX_SPEED) + (kd * deriv * Globals.MAX_SPEED);
                        turn = Math.max(Math.abs(pow), Globals.MIN_SPEED) * Math.signum(pow);
                    }
                }
            }
        }
        double scaleFactor;
        double max = Math.max(Math.abs(drive - angle), Math.abs(drive + angle));
        if(Globals.MAX_SPEED > max) {
            if(angle * 2 > drive) {
                angle *= Globals.MAX_SPEED / max;
            }
            drive *= Globals.MAX_SPEED / max;
        }
        max = Math.max(Math.abs(drive + turn - angle), Math.max(Math.abs(drive - turn + angle), Math.max(Math.abs((drive + turn + angle)), Math.abs((drive - turn - angle)))));
        scaleFactor = Globals.MAX_SPEED / max;
        odometry.update(data, current);
        robot.setDrivePower(scaleFactor * (drive + turn - angle), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle)); // Set motors to values based on gamepad
    }

    /*
    private double adjust(double varToAdjust) { // Square-root driving
        if (varToAdjust < 0) {
            varToAdjust = -Math.sqrt(-varToAdjust);
        } else {
            varToAdjust = Math.sqrt(varToAdjust);
        }
        return varToAdjust;
    }
     */

    public double adjust(double varToAdjust) {
        return (Math.atan(5 * varToAdjust) / Math.atan(5));
    }

}