package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Circle;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Functions;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Line;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.Unit;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

public class Odometry implements Subsystem {

    private static GOFHardware robot;
    public static double lastAngle;
    public double lastXPos = 0;
    public double lastYPos = 0;
    private static double angle;
    private double x = Globals.START_X;
    private double y = Globals.START_Y;
    private double lastTime = System.currentTimeMillis();
    private double dTime = 0;
    public int updates = 0;
    private boolean update = true;
    private static Odometry thismetry = null;
    private static double angleOffset = 0;
    private double velocity = 0;

    RevBulkData data;

    private State state = State.ON;

    public Odometry(GOFHardware robot) {
        this.data = robot.bulkRead();
    }

    public static Odometry getInstance(GOFHardware myRobot) {
        robot = myRobot;
        if(thismetry == null) {
            lastAngle = robot.getAngle();
            angle = robot.getAngle();
            robot.resetOmnis();
            thismetry = new Odometry(myRobot);
        }
        else {
            angleOffset = Functions.normalize(lastAngle - Globals.START_THETA);
        }
        thismetry.setState(State.ON);
        return thismetry;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return Functions.normalize(robot.getAngle() + angleOffset);
    }

    public Point getPoint() {
        return new Point(x, y);
    }

    public boolean isUpdating() {
        return update && state == State.ON;
    }

    public double getUpdateTime() {
        return dTime / (1000 * updates);
    }

    public void update(RevBulkData data1) {
        if(update) {
            data = data1;
            updates++;
            angle = Functions.normalize(robot.getAngle() + angleOffset);
            handler.pushData("Angle", angle);
            double dTheta = angle - lastAngle;
            if(dTheta >= 300) {
                dTheta -= 360;
            }
            if(dTheta <= -300) {
                dTheta += 360;
            }
            lastAngle = angle;
            double xDist;
            double yDist;
            if(data != null && robot.rf != null && robot.rb != null) {
                xDist = (-data.getMotorCurrentPosition(robot.rb) - lastXPos) * Globals.OMNI_FEET_PER_TICK;
                yDist = (-data.getMotorCurrentPosition(robot.rf) - lastYPos) * Globals.OMNI_FEET_PER_TICK;
            }
            else {
                return;
            }
            xDist -= ((Globals.xOffset / 12f) * Math.toRadians(dTheta));
            yDist -= ((Globals.yOffset / 12f) * Math.toRadians(dTheta));
            double displacement = Math.hypot(xDist, yDist);
            angle += Math.toDegrees(Math.atan2(yDist, xDist)) - (dTheta / 2);
            if (dTheta != 0 && displacement > 0) {
                double radius = Math.abs((((360.0 / dTheta) * displacement) / (2.0 * Math.PI)));
                Circle myCircle = new Circle(new Point(x, y), (((2 * radius * Math.sin(Math.toRadians(dTheta) / 2)))));
                Line myLine = new Line(new Point(x, y), new Point(x + 1, y + Math.tan(Math.toRadians(angle))));
                Point[] points;
                try {
                    if(Math.abs(Math.cos(Math.toRadians(angle))) <= 0.05) {
                        Log.d(TAG, "update: \"You have engaged in a movement incompatible with this robot.  If you do that again, prepare to die.\"");
                        throw new GOFException("You have engaged in a movement incompatible with this robot.  If you do that again, prepare to die.");
                    }
                    Object[] objs = Functions.infiniteLineCircleIntersection(myCircle, myLine).toArray();
                    points = new Point[objs == null ? 0 : objs.length];
                    int co = 0;
                    if(objs != null) {
                        for(Object obj : objs) {
                            if(obj instanceof Point) {
                                points[co] = (Point)obj;
                            }
                            co++;
                        }
                    }
                }
                catch(Exception p_exception) {
                    points = new Point[0];
                }
                Point approxPoint = new Point(x + (displacement * Math.cos(Math.toRadians(angle))), y + (displacement * Math.sin(Math.toRadians(angle))));
                if(points.length == 0) {
                    x += displacement * Math.cos(Math.toRadians(angle));
                    y += displacement * Math.sin(Math.toRadians(angle));
                }
                else if(points.length == 1) {
                    x = points[0].getX();
                    y = points[0].getY();
                }
                else {
                    Point testPoint = points[0];
                    Point testPoint2 = points[1];
                    double dist1 = testPoint.distance(approxPoint, Unit.FEET);
                    double dist2 = testPoint2.distance(approxPoint, Unit.FEET);
                    if(Math.abs(dist1) < Math.abs(dist2)) {
                        x = points[0].getX();
                        y = points[0].getY();
                    }
                    else {
                        x = points[1].getX();
                        y = points[1].getY();
                    }
                }
            }
            else {
                x += displacement * Math.cos(Math.toRadians(angle));
                y += displacement * Math.sin(Math.toRadians(angle));
            }
            if(data != null && robot.rf != null && robot.rb != null) {
                lastXPos = -data.getMotorCurrentPosition(robot.rb);
                lastYPos = -data.getMotorCurrentPosition(robot.rf);
            }
            else {
                return;
            }
            dTime += System.currentTimeMillis() - lastTime;
            velocity = displacement / (System.currentTimeMillis() - lastTime);
            lastTime = System.currentTimeMillis();
            //y *= -1;
            handler.pushData("Odometry Point", new Point(x, y));
        }
    }

    public void update(RevBulkData data1, double angle) {
        if(update) {
            data = data1;
            updates++;
            handler.pushData("Angle", angle);
            double dTheta = angle - lastAngle;
            if(dTheta >= 300) {
                dTheta -= 360;
            }
            if(dTheta <= -300) {
                dTheta += 360;
            }
            lastAngle = angle;
            double xDist;
            double yDist;
            if(data != null && robot.rf != null && robot.rb != null) {
                xDist = (-data.getMotorCurrentPosition(robot.rb) - lastXPos) * Globals.OMNI_FEET_PER_TICK;
                yDist = (-data.getMotorCurrentPosition(robot.rf) - lastYPos) * Globals.OMNI_FEET_PER_TICK;
            }
            else {
                return;
            }
            xDist -= ((Globals.xOffset / 12f) * Math.toRadians(dTheta));
            yDist -= ((Globals.yOffset / 12f) * Math.toRadians(dTheta));
            double displacement = Math.hypot(xDist, yDist);
            angle += Math.toDegrees(Math.atan2(yDist, xDist)) - (dTheta / 2);
            // double arcAngle = Math.toRadians(angle + Math.toDegrees(Math.atan2(yDist, xDist)));
            double x1 = displacement * Math.cos(Math.toRadians(angle));
            double y1 = displacement * Math.sin(Math.toRadians(angle));
            if (dTheta != 0 && displacement > 0) {
                /*
                CircleCircleIntersection intersection = new CircleCircleIntersection(new Circle(new Vector2(x, y), displacement), new Circle(new Vector2(0, 0), (Math.sqrt(Math.pow(displacement, 2) / (4 * Math.pow(Math.sin(0.5 * dTheta), 2))))));
                double direction = Math.signum(displacement * Math.cos(Math.toRadians(angle)));
                Vector2[] points = intersection.getIntersectionPoints();
                */
                double radius = Math.abs((((360.0 / dTheta) * displacement) / (2.0 * Math.PI)));
                Circle myCircle = new Circle(new Point(this.x, this.y), (((2 * radius * Math.sin(Math.toRadians(dTheta) / 2)))));
                Line myLine = new Line(new Point(this.x, this.y), new Point(this.x + 1, this.y + Math.tan(Math.toRadians(angle))));
                Point[] points;
                try {
                    if(Math.abs(Math.cos(Math.toRadians(angle))) <= 0.05) {
                        Log.d(TAG, "update: \"You have engaged in a movement incompatible with this robot.  If you do that again, prepare to die.\"");
                        throw new GOFException("You have engaged in a movement incompatible with this robot.  If you do that again, prepare to die.");
                    }
                    Object[] objs = Functions.infiniteLineCircleIntersection(myCircle, myLine).toArray();
                    points = new Point[objs == null ? 0 : objs.length];
                    int co = 0;
                    if(objs != null) {
                        for(Object obj : objs) {
                            if(obj instanceof Point) {
                                points[co] = (Point)obj;
                            }
                            co++;
                        }
                    }
                }
                catch(Exception p_exception) {
                    points = new Point[0];
                }
                Point approxPoint = new Point(this.x + x1, this.y + y1);
                if(points.length == 0) {
                    this.x += x1;
                    this.y += y1;
                }
                else if(points.length == 1) {
                    this.x = points[0].getX();
                    this.y = points[0].getY();
                }
                else {
                    Point testPoint = points[0];
                    Point testPoint2 = points[1];
                    double dist1 = testPoint.distance(approxPoint, Unit.FEET);
                    double dist2 = testPoint2.distance(approxPoint, Unit.FEET);
                    if(Math.abs(dist1) < Math.abs(dist2)) {
                        this.x = points[0].getX();
                        this.y = points[0].getY();
                    }
                    else {
                        this.x = points[1].getX();
                        this.y = points[1].getY();
                    }
                }
            }
            else {
                this.x += x1;
                this.y += y1;
            }
            if(data != null && robot.rf != null && robot.rb != null) {
                lastXPos = -data.getMotorCurrentPosition(robot.rb);
                lastYPos = -data.getMotorCurrentPosition(robot.rf);
            }
            else {
                return;
            }
            dTime += System.currentTimeMillis() - lastTime;
            velocity = displacement / (System.currentTimeMillis() - lastTime);
            lastTime = System.currentTimeMillis();
            handler.pushData("Odometry Point", new Point(x, y));
        }

        /*
        if(update) {
            updates++;
            angle = robot.getAngle() + angleOffset;
            double dTheta = angle - lastAngle;
            if(dTheta >= 300) {
                dTheta -= 360;
            }
            if(dTheta <= -300) {
                dTheta += 360;
            }
            lastAngle = angle;
            double xDist = (robot.getHOmniPos() - lastXPos) * Globals.OMNI_FEET_PER_TICK;
            double yDist = (robot.getVOmniPos() - lastYPos) * Globals.OMNI_FEET_PER_TICK;
            double displacement = Math.hypot(xDist, yDist);
            angle += Math.toDegrees(Math.atan2(yDist, xDist));
            // double arcAngle = Math.toRadians(angle + Math.toDegrees(Math.atan2(yDist, xDist)));
            if (dTheta != 0 && displacement > 0) {
                CircleCircleIntersection intersection = new CircleCircleIntersection(new Circle(new Vector2(x, y), displacement), new Circle(new Vector2(0, 0), (Math.sqrt(Math.pow(displacement, 2) / (4 * Math.pow(Math.sin(0.5 * dTheta), 2))))));
                double direction = Math.signum(displacement * Math.cos(Math.toRadians(angle)));
                Vector2[] points = intersection.getIntersectionPoints();
                if(points.length == 0) {
                    x += displacement * Math.cos(Math.toRadians(angle));
                    y += displacement * Math.sin(Math.toRadians(angle));
                }
                else if(points.length == 1) {
                    x = points[0].x;
                    y = points[0].y;
                }
                else if (direction > 0) {
                    if(points[0].x >= x) {
                        x = points[0].x;
                        y = points[0].y;
                    }
                    else {
                        x = points[1].x;
                        y = points[1].y;
                    }
                }
                else if (direction < 0) {
                    if(points[0].x >= x) {
                        x = points[1].x;
                        y = points[1].y;
                    }
                    else {
                        x = points[0].x;
                        y = points[0].y;
                    }
                }
                else {
                    direction = Math.signum(displacement * Math.sin(Math.toRadians(angle)));
                    if(direction > 0) {
                        if(points[0].y >= y) {
                            x = points[0].x;
                            y = points[0].y;
                        }
                        else {
                            x = points[1].x;
                            y = points[1].y;
                        }
                    }
                    else if(direction < 0) {
                        if(points[0].y <= y) {
                            x = points[0].x;
                            y = points[0].y;
                        }
                        else {
                            x = points[1].x;
                            y = points[1].y;
                        }
                    }
                    else {
                        x = x;
                        y = y;
                    }
                }
            }
            else {
                x += displacement * Math.cos(Math.toRadians(angle));
                y += displacement * Math.sin(Math.toRadians(angle));
            }
            lastXPos = robot.getHOmniPos();
            lastYPos = robot.getVOmniPos();
            dTime += System.currentTimeMillis() - lastTime;
            lastTime = System.currentTimeMillis();
        }
        */
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, RevBulkData data1, RevBulkData data2, Odometry odometry) {
        if(gamepad1.x) {
            angleOffset = Functions.normalize(90 - robot.getAngle());
            /*
            double lf = getPoint().distance(-5.25, -5.25, Unit.FEET);
            double rf = getPoint().distance(-0.75, -5.25, Unit.FEET);
            double lb = getPoint().distance(-5.25, 5.25, Unit.FEET);
            double rb = getPoint().distance(-0.75, 5.25, Unit.FEET);
            if(Math.abs(lf) <= Math.abs(rf) && Math.abs(lf) <= Math.abs(lb) && Math.abs(lf) <= Math.abs(rb)) {
                x = -5.25;
                y = -5.25;
            }
            else if(Math.abs(rf) <= Math.abs(lf) && Math.abs(rf) <= Math.abs(lb) && Math.abs(rf) <= Math.abs(rb)) {
                x = -0.75;
                y = -5.25;
            }
            else if(Math.abs(lb) <= Math.abs(lf) && Math.abs(lb) <= Math.abs(rf) && Math.abs(lb) <= Math.abs(rb)) {
                x = -5.25;
                y = 5.25;
            }
            else {
                x = -0.75;
                y = 5.25;
            }
        }
             */
            x = 5.25;
            y = 5.25;
            lastXPos = -data.getMotorCurrentPosition(robot.rb);
            lastYPos = -data.getMotorCurrentPosition(robot.rf);
            lastAngle = 90;
            angle = 90;
            update(data1);
        }
        update(data1);
    }

    public void reset() {
        angleOffset = 0;
        robot.resetOmnis();
        lastXPos = 0;
        lastYPos = 0;
        x = Globals.START_X;
        y = Globals.START_Y;
        lastTime = System.currentTimeMillis();
        dTime = 0;
        lastAngle = robot.getAngle();
        angle = robot.getAngle();
        thismetry = new Odometry(robot);
        robot.resetOmnis();
    }

    public void load(double angle) {
        if(handler.contains("Odometry Point")) {
            if(handler.getData("Odometry Point") instanceof Point) {
                Point point = (Point)handler.getData("Odometry Point");
                x = point.getX();
                y = point.getY();
            }

            angleOffset = angle - robot.getAngle();
            data = robot.bulkRead();
            lastXPos = -data.getMotorCurrentPosition(robot.rb);
            lastYPos = -data.getMotorCurrentPosition(robot.rf);
        }
    }

    public double getVelocity() {
        return velocity;
    }

    public void setState(State newState) {
        this.state = newState;
    }
}
