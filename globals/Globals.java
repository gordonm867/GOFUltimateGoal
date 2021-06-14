package org.firstinspires.ftc.teamcode.gofultimategoal.globals;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Globals {
    public static final double DRIVE_RADIUS = 4 * Math.PI; // Drive wheel radius
    public static final double OMNI_RADIUS = 1.5 * Math.PI; // Omni wheel radius

    public static final double DRIVE_FEET_PER_TICK = (DRIVE_RADIUS / (560 * 12)); // Feet moved per drive wheel encoder tick
    public static final double OMNI_FEET_PER_TICK = (OMNI_RADIUS / (1440 * 12)); // Feet moved per omni wheel encoder tick

    public static double MAX_SPEED = 1.0; // Maximum proportion of speed allowed
    public static double MIN_SPEED = 0.18;

    public static double MAX_IN_SPEED = 1.0;
    public static double MIN_IN_SPEED = 0.45;

    public static double MAX_WOBBLE = 0.5;

    public static double START_Y = -5.25853; // Default starting y position
    public static double START_X = -2.98; // Default starting x position
    public static double START_THETA = 90; // Default starting angle

    public static double xOffset = -6.8520357496;
    public static double yOffset = -0.29791459764;

    public static double MIN_B = 0;
    public static double MIN_G = 80;
    public static double MIN_R = 100;

    public static double MAX_B = 50;
    public static double MAX_G = 255;
    public static double MAX_R = 255;

    public static double RING_SIZE = 105;

    public static double BLUR_CONSTANT = 5;

    public static boolean AUTO = false;

    public static double MAX_STRAFE = 4.0;
    public static double MAX_STRAIGHT = 6.0;

    public static double MAX_ACCEL = 6.3;

    //-48º, xraw = -1774, yraw = -407

}