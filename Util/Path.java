package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Line;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;

import java.util.ArrayList;

public class Path {

    public ArrayList<Point> path;
    public ArrayList<Obstacle> obstacles;
    int index = 0;

    public boolean done = false;

    public Path(ArrayList<Point> path, ArrayList<Obstacle> obstacles) {
        this.path = path;
        this.obstacles = obstacles;
    }

    public Point getNext() {
        if(index < path.size()) {
            return path.get(index);
        }
        done = true;
        return path.get(path.size() - 1);
    }

    public Line getLine() {
        try {
            return new Line(path.get(index), path.get(index + 1));
        }
        catch(Exception e) {
            return null;
        }
    }

    public void advance() {
        index++;
        if(index == path.size()) {
            done = true;
        }
    }

    public void reset() {
        index = 0;
    }
}
