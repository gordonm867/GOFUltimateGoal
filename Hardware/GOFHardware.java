package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Functions;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

/**
 * HUB 2
 *  MOTORS
 *   • ex
 *   • lw
 *   • vodo
 *   • hodo
 *  SERVOS
 *   •
 *   •
 *   •
 *   •
 *  ANALOG
 *   •
 *   •
 *  I2C 0
 *   • Port 0: g0
 *
 * HUB 3
 *  MOTORS
 *   • lf
 *   • rf
 *   • lb
 *   • rb
 *  SERVOS
 *   • fm1
 *   • cl (clamp open/close)
 *   • mc (clamp slide)
 *   • fm2
 *   • odo
 *   • cap
 *  ANALOG
 */
public class GOFHardware {
    /* Declare OpMode members */
    public BNO055IMU gyro;

    public DcMotor rf;
    public DcMotor rb;
    public DcMotor lf;
    public DcMotor lb;

    public boolean enabled;

    public ExpansionHubEx ex2;
    public ExpansionHubEx ex3;

    private static GOFHardware myInstance = null;


    /* Constructor */
    public static GOFHardware getInstance() {
        if (myInstance == null) {
            myInstance = new GOFHardware();
        }
        myInstance.enabled = true;
        return myInstance;
    }

    /**
     * Initialize robot hardware
     *
     * @param hwMap OpMode's internal HardwareMap
     */
    public void init(HardwareMap hwMap) {
        try {
            ex2 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        } catch (Exception p_exception) {
            ex2 = null;
        }
        try {
            ex3 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        } catch (Exception p_exception) {
            ex3 = null;
        }
        try { // Gyro
            gyro = hwMap.get(BNO055IMU.class, "g0");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            if (ex2 != null) {
                gyro = LynxOptimizedI2cFactory.createLynxEmbeddedImu(ex2.getStandardModule(), 0);
            }
            gyro.initialize(parameters);
        } catch (Exception p_exception) {
            gyro = null;
        }
        try { // Left rear wheel
            lb = hwMap.get(DcMotor.class, "lb");
            lb.setDirection(DcMotor.Direction.REVERSE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setPower(0);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            lb = null;
        }

        try { // Left front wheel
            lf = hwMap.get(DcMotor.class, "lf");
            lf.setDirection(DcMotor.Direction.REVERSE);
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lf.setPower(0);
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            lf = null;
        }

        try { // Right rear wheel
            rb = hwMap.get(DcMotor.class, "rb");
            rb.setDirection(DcMotor.Direction.FORWARD);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setPower(0);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            rb = null;
        }

        try { // Right front wheel
            rf = hwMap.get(DcMotor.class, "rf");
            rf.setDirection(DcMotor.Direction.FORWARD);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setPower(0);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            rf = null;
        }

    }

    /**
     * Get robot's angle
     *
     * @return Robot angle
     */
    public double getAngle() {
        return Functions.normalize(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + Globals.START_THETA);
    }

    /**
     * Get robot's angle
     *
     * @return Robot angle
     */
    public double getYAngle() {
        return Functions.normalize(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle + Globals.START_THETA);
    }

    /**
     * Get robot's angle
     *
     * @return Robot angle
     */
    public double getXAngle() {
        return Functions.normalize(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle + Globals.START_THETA);
    }

    /**
     * Get bulk data
     *
     * @return REV Hub bulk data
     */
    public RevBulkData bulkRead() {
        if (ex2 != null) {
            return ex2.getBulkInputData();
        }
        return null;
    }

    /**
     * Get secondary bulk data
     *
     * @return REV Hub bulk data
     */
    public RevBulkData bulkReadTwo() {
        if (ex3 != null) {
            return ex3.getBulkInputData();
        }
        return null;
    }

    /**
     * Drive with specified wheel powers
     *
     * @param leftBackDrivePower   Power [-1, 1] for left rear drive wheel
     * @param leftFrontDrivePower  Power [-1, 1] for left front drive wheel
     * @param rightBackDrivePower  Power [-1, 1] for right rear drive wheel
     * @param rightFrontDrivePower Power [-1, 1] for right front drive wheel
     */
    public void setDrivePower(double leftBackDrivePower, double leftFrontDrivePower, double rightBackDrivePower, double rightFrontDrivePower) { // Send power to wheels
        if (enabled) {
            /*
            if(Math.abs(leftBackDrivePower) < Globals.MIN_SPEED && leftBackDrivePower != 0) {
                leftBackDrivePower = Globals.MIN_SPEED * Math.signum(leftBackDrivePower);
            }
            if(Math.abs(leftFrontDrivePower) < Globals.MIN_SPEED && leftFrontDrivePower != 0) {
                leftFrontDrivePower = Globals.MIN_SPEED * Math.signum(leftFrontDrivePower);
            }
            if(Math.abs(rightBackDrivePower) < Globals.MIN_SPEED && rightBackDrivePower != 0) {
                rightBackDrivePower = Globals.MIN_SPEED * Math.signum(rightBackDrivePower);
            }
            if(Math.abs(rightFrontDrivePower) < Globals.MIN_SPEED && rightFrontDrivePower != 0) {
                rightFrontDrivePower = Globals.MIN_SPEED * Math.signum(rightFrontDrivePower);
            }
             */
            if (lb != null) {
                leftBackDrivePower = Range.clip(leftBackDrivePower, -Globals.MAX_SPEED, Globals.MAX_SPEED);
                lb.setPower(leftBackDrivePower);
            }
            if (lf != null) {
                leftFrontDrivePower = Range.clip(leftFrontDrivePower, -Globals.MAX_SPEED, Globals.MAX_SPEED);
                lf.setPower(leftFrontDrivePower);
            }
            if (rb != null) {
                rightBackDrivePower = Range.clip(rightBackDrivePower, -Globals.MAX_SPEED, Globals.MAX_SPEED);
                rb.setPower(rightBackDrivePower);
            }
            if (rf != null) {
                rightFrontDrivePower = Range.clip(rightFrontDrivePower, -Globals.MAX_SPEED, Globals.MAX_SPEED);
                rf.setPower(rightFrontDrivePower);
            }
        }
    }

    /**
     * Reset omni encoders
     */
    public void resetOmnis() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Get current encoder reading of vertically aligned omni
     *
     * @return Encoder reading
     */
    public int getVOmniPos(RevBulkData rev) {
        if (lf != null) {
            return rev.getMotorCurrentPosition(lf);
        }
        return 0;
    }

    /**
     * Get current encoder reading of horizontally aligned omni
     *
     * @return Encoder reading
     */
    public int getHOmniPos(RevBulkData rev) {
        if (lb != null) {
            return -rev.getMotorCurrentPosition(lb);
        }
        return 0;
    }
}