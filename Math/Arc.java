package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.Unit;

public class Arc extends Line {
    public Point end1;
    public Point end2;
    public Point center;
    public double radius;

    public Arc(Point end1, Point end2, Point center) {
        super(end1, end2);
        this.end1 = end1;
        this.end2 = end2;
        this.center = center;
        this.radius = end1.distance(center, Unit.FEET);
    }

    public double getLength() {
        double a1 = end1.angle(center, AngleUnit.RADIANS);
        double a2 = end2.angle(center, AngleUnit.RADIANS);
        return Math.abs(a1 - a2) * radius;
    }

    @Override
    public String toString() {
        return "Arc from " + end1.toString() + " to " + end2.toString() + " along a circle of center " + center.toString();
    }
}
