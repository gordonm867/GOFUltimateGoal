package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math;

public final class Circle {

    public final double r;
    private Point center;

    public Circle(Point center, double radius) {
        this.center = center;
        this.r = radius;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        Circle other = (Circle)obj;
        if (center == null) {
            if (other.center != null)
                return false;
        }
        else if (!center.equals(other.center)) {
            return false;
        }
        if (Double.doubleToLongBits(r) != Double.doubleToLongBits(other.r)) {
            return false;
        }
        return true;
    }

    public Point getCenter() {
        return center;
    }

    public double getRadius() {
        return r;
    }

    public double getArea() {
        return Math.pow(r, 2) * Math.PI;
    }

    public double getCircumference() {
        return 2 * Math.PI * r;
    }

}
