package org.firstinspires.ftc.teamcode.gofultimategoal.util;

import org.firstinspires.ftc.teamcode.gofultimategoal.math.Circle;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;

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
