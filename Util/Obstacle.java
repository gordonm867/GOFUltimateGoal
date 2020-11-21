package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Circle;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;

public class Obstacle {
    public double x;
    public double y;
    public double radius;

    public Obstacle(double x, double y, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
    }

    public Circle getCircle() {
        return new Circle(new Point(x, y), radius);
    }
}
