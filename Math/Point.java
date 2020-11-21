package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.Unit;

public class Point {

    private double x;
    private double y;
    private double angle;
    private double speed;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
        this.angle = Double.NaN;
        this.speed = Double.NaN;
    }

    public Point(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.speed = Double.NaN;
    }

    public Point(double x, double y, double angle, double speed) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.speed = speed;
    }

    /* To get a Point with an x, y, and speed, pass Double.NaN as angle :) */

    public double distance(double newX, double newY, Unit unit) {
        if(unit == Unit.TICKS) {
            return (int) (Math.round((Math.sqrt(Math.pow(x - newX, 2) + Math.pow(y - newY, 2))) / -Globals.DRIVE_FEET_PER_TICK));
        }
        else if(unit == Unit.FEET) {
            return Math.sqrt(Math.pow(x - newX, 2) + Math.pow(y - newY, 2));
        }
        else {
            return Math.sqrt(Math.pow(x - newX, 2) + Math.pow(y - newY, 2)) / 12.0;

        }
    }

    public double distance(Point newPoint, Unit unit) {
        if(unit == Unit.TICKS) {
            return (int) (Math.round((Math.sqrt(Math.pow(x - newPoint.getX(), 2) + Math.pow(y - newPoint.getY(), 2))) / -Globals.DRIVE_FEET_PER_TICK));
        }
        else if(unit == Unit.FEET) {
            return Math.sqrt(Math.pow(x - newPoint.getX(), 2) + Math.pow(y - newPoint.getY(), 2));
        }
        else {
            return Math.sqrt(Math.pow(x - newPoint.getX(), 2) + Math.pow(y - newPoint.getY(), 2)) / 12.0;

        }
    }

    public double angle(double newX, double newY, AngleUnit unit) {
        Point point = new Point(newX, newY);
        return angle(point, unit);
    }

    public double angle(Point newPoint, AngleUnit unit) {
        double angle;
        if(unit == AngleUnit.DEGREES) {
            angle = Functions.normalize(Math.toDegrees(Math.atan2(newPoint.getY() - y, newPoint.getX() - x)));
            while(angle < -180 || angle > 180) {
                if(angle < -180) {
                    angle += 360;
                }
                else {
                    angle -= 360;
                }
            }
        }
        else {
            angle = Math.atan2(newPoint.getY() - y, newPoint.getX() - x);
            while(angle < -Math.PI || angle > Math.PI) {
                if(angle < -Math.PI) {
                    angle += 2 * Math.PI;
                }
                else {
                    angle -= 2 * Math.PI;
                }
            }
        }
        return angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return angle;
    }

    public void setX(double newX) {
        x = newX;
    }

    public void setY(double newY) {
        y = newY;
    }

    public void setTheta(double newO) {
        angle = newO;
    }

    @NonNull
    public String toString() {
        return "(" + getX() + ", " + getY() + ")";
    }

    @Override
    public boolean equals(Object o) {
        if(!(o instanceof Point)) {
            return false;
        }
        if(((Point)o).getX() == getX() && ((Point)o).getY() == getY()) {
            return true;
        }
        return false;
    }
}