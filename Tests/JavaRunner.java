package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Functions;

public class JavaRunner {
    public static void main(String[] args) {
        System.out.println(Functions.calcTargetVel(11, 26.0/12.0, 26, AngleUnit.DEGREES)); // 0.225
        System.out.println(Functions.calcTargetVel(6, 26.0/12.0, 26, AngleUnit.DEGREES)); // 0.20
    }
}
