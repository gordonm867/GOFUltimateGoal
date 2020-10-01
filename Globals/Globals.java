package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals;

public final class Globals {
    public static final double DRIVE_RADIUS = 4 * Math.PI; // Drive wheel radius
    public static final double OMNI_RADIUS = 3 * Math.PI; // Omni wheel radius

    public static final double DRIVE_FEET_PER_TICK = (DRIVE_RADIUS / (560 * 12)); // Feet moved per drive wheel encoder tick
    public static final double OMNI_FEET_PER_TICK = (OMNI_RADIUS / (1440 * 12)); // Feet moved per omni wheel encoder tick

    public static double MAX_SPEED = 1.0; // Maximum proportion of speed allowed
    public static double MIN_SPEED = 0.35;

    public static double START_X = 0; // Default starting x position
    public static double START_Y = 0; // Default starting y position
    public static double START_THETA = 180; // Default starting angle

    public static double xOffset = 2.175;
    public static double yOffset = 1.9125;

}