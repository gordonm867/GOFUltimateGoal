package org.firstinspires.ftc.teamcode.gofultimategoal.math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Obstacle;

import java.util.ArrayList;

public class Line {

    private Point end1;
    private Point end2;

    public ArrayList<Obstacle> obstacles;

    public Line(Point end1, Point end2) {
        this.end1 = end1;
        this.end2 = end2;
        obstacles = null;
    }

    public Line(Point end1, Point end2, ArrayList<Obstacle> obstacles) {
        this.end1 = end1;
        this.end2 = end2;
        this.obstacles = obstacles;
    }

    public double getXComp() {
        return(end2.getX() - end1.getX());
    }

    public double getYComp() {
        return(end2.getY() - end1.getY());
    }

    public double getLength() {
        return(Math.sqrt(Math.pow(getXComp(), 2) + Math.pow(getYComp(), 2)));
    }

    public double getAngle(AngleUnit unit) {
        if(unit == AngleUnit.DEGREES) {
            return Math.toDegrees(Math.atan2(getYComp(), getXComp()));
        }
        return Math.atan2(getYComp(), getXComp());
    }

    public double getSlope() {
        return getYComp() / getXComp();
    }

    public double getYInt() {
        return(end1.getY() - (getSlope() * end1.getX()));
    }

    public Point getPoint1() {
        return end1;
    }

    public Point getPoint2() {
        return end2;
    }

    public Point getMidpoint() {
        return new Point((end1.getX() + end2.getX()) / 2, (end1.getY() + end2.getY()) / 2);
    }

    @Override
    public String toString() {
        return "Line from " + end1.toString() + " to " + end2.toString();
    }
}