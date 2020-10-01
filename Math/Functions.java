package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math;

import java.util.ArrayList;
import static java.lang.Math.*;

public class Functions {

    /**
     * Finds every intersection between a line segment and a circle
     * @param circle Circle to test intersection points
     * @param line Line segment to test intersection points
     * @return ArrayList of all intersection points along the line segment and the circle
     */
    public static ArrayList<Point> lineCircleIntersection(Circle circle, Line line) {
        if (abs(line.getYComp()) < 0.003) {
            if (line.getYComp() >= 0) {
                line = new Line(new Point(line.getPoint1().getX(), line.getPoint1().getY() + 0.003), line.getPoint2());
            } else {
                line = new Line(new Point(line.getPoint1().getX(), line.getPoint1().getY() - 0.003), line.getPoint2());
            }
        }
        if (abs(line.getXComp()) < 0.003) {
            if (line.getXComp() >= 0) {
                line = new Line(new Point(line.getPoint1().getX() + 0.003, line.getPoint1().getY()), line.getPoint2());
            } else {
                line = new Line(new Point(line.getPoint1().getX() - 0.03, line.getPoint2().getY()), line.getPoint2());
            }
        }
        double slope = line.getSlope();
        double yint = line.getYInt();
        double a = 1 + pow(slope, 2);
        double b = (-2 * circle.getCenter().getX()) + (2 * slope * yint) - (2 * slope * circle.getCenter().getY());
        double c = pow(circle.getCenter().getX(), 2) + pow(yint, 2) - (2 * yint * circle.getCenter().getY()) + pow(circle.getCenter().getY(), 2) - pow(circle.getRadius(), 2);
        ArrayList<Point> allPoints = new ArrayList<>();
        try {
            double constant = sqrt(pow(b, 2) - (4 * a * c));
            double quad = ((-b + constant) / (2 * a));
            allPoints.add(new Point(quad, (slope * quad) + yint));
            quad = ((-b - constant) / (2 * a));
            allPoints.add(new Point(quad, (slope * quad) + yint));
            double minX = min(line.getPoint1().getX(), line.getPoint2().getX());
            double maxX = max(line.getPoint1().getX(), line.getPoint2().getX());
            for (int x = allPoints.size() - 1; x >= 0; x--) {
                if (minX > allPoints.get(x).getX() || maxX < allPoints.get(x).getX() || Double.isNaN(allPoints.get(x).getX()) || Double.isInfinite(allPoints.get(x).getX())) {
                    allPoints.remove(x);
                }
            }
        } catch (Exception p_exception) {
        }
        return allPoints;
    }

    /**
     * Finds every intersection points between a line and a circle
     * @param circle Circle to test intersections
     * @param line Line to test intersections
     * @return ArrayList of all intersection points
     */
    public static ArrayList<Point> infiniteLineCircleIntersection(Circle circle, Line line) {
        if (abs(line.getYComp()) < 0.003) {
            if (line.getYComp() >= 0) {
                line = new Line(new Point(line.getPoint2().getX(), line.getPoint2().getY() - 0.003), line.getPoint2());
            } else {
                line = new Line(new Point(line.getPoint2().getX(), line.getPoint2().getY() + 0.003), line.getPoint2());
            }
        }
        if (abs(line.getXComp()) < 0.003) {
            if (line.getXComp() >= 0) {
                line = new Line(new Point(line.getPoint2().getX() - 0.003, line.getPoint1().getY()), line.getPoint2());
            } else {
                line = new Line(new Point(line.getPoint2().getX(), line.getPoint2().getY() + 0.003), line.getPoint2());
            }
        }
        double slope = line.getSlope();
        double yint = line.getYInt();
        double a = 1 + pow(slope, 2);
        double b = (-2 * circle.getCenter().getX()) + (2 * slope * yint) - (2 * slope * circle.getCenter().getY());
        double c = pow(circle.getCenter().getX(), 2) + pow(yint, 2) - (2 * yint * circle.getCenter().getY()) + pow(circle.getCenter().getY(), 2) - pow(circle.getRadius(), 2);
        ArrayList<Point> allPoints = new ArrayList<>();
        try {
            double constant = sqrt(pow(b, 2) - (4 * a * c));
            double quad = ((-b + constant) / (2 * a));
            if(!((Double)quad).isNaN()) {
                allPoints.add(new Point(quad, (slope * quad) + yint));
            }
            quad = ((-b - constant) / (2 * a));
            if(!((Double)quad).isNaN()) {
                allPoints.add(new Point(quad, (slope * quad) + yint));
            }
        }
        catch (Exception p_exception) {}
        return allPoints;
    }

    /**
     * Determine whether or not robot has passed a point on a line
     * @param myLine Line being followed
     * @param myPoint Current point
     * @param testPoint Target point
     * @return Boolean representing whether or not the robot has passed the target
     */
    public static boolean isPassed(Line myLine, Point myPoint, Point testPoint) {
        if(myLine.getYComp() == 0) {
            return myPoint.getX() * signum(myLine.getXComp()) >= testPoint.getX() * signum(myLine.getXComp());
        }
        return !(signum(myLine.getYComp()) * (myPoint.getY() - testPoint.getY()) < signum(myLine.getYComp()) * ((-1 / myLine.getSlope()) * (myPoint.getX() - testPoint.getX())));
    }

    /**
     * Method to take the mathematical modulus of two numbers, rather than the default but sometimes inaccurate built-in modulus function
     * @param s1 First argument (s1 % s2)
     * @param s2 Second argument (s1 % s2)
     * @return True modulus of s1 and s2
     */
    public static double trueMod(double s1, double s2) {
        return(s1 % s2 + s2) % s2;
    }

    public static double normalize(double angle) {
        angle = trueMod(angle, 360.0);
        angle -= angle > 180 ? 360 : angle < -180 ? -360 : 0;
        return angle;
    }
}