package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;

public class Point {

    private double x;
    private double y;
    private double angle;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
        this.angle = Double.NaN;
    }

    public Point(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public double distance(double newX, double newY) {
        return(int)(Math.round((Math.sqrt(Math.pow(x - newX, 2) + Math.pow(y - newY, 2))) / -Globals.DRIVE_FEET_PER_TICK));
    }

    public int distance(Point newPoint) {
        return(int)(Math.round((Math.sqrt(Math.pow(x - newPoint.getX(), 2) + Math.pow(y - newPoint.getY(), 2))) / -Globals.DRIVE_FEET_PER_TICK));
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
        return "(" + Math.round(100 * getX()) / 100f + ", " + Math.round(100 * getY()) / 100f + ")";
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