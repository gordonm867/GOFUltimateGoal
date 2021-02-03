package org.firstinspires.ftc.teamcode.gofultimategoal.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
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

//yraw is in port 2, xraw is in port 0

public class GOFHardware {
    /* Declare OpMode members */
    public BNO055IMU gyro;

    public DcMotor rf;
    public DcMotor rb;
    public DcMotor lf;
    public DcMotor lb;
    public DcMotor in;
    public DcMotor shoot1;
    public DcMotor shoot2;
    public DcMotor wobblewheel;

    public Servo wobble;
    public Servo flicker;

    public boolean enabled;

    public ExpansionHubEx ex2;
    public ExpansionHubEx ex3;

    private static GOFHardware myInstance = null;

    public OpenCvCamera phoneCam;

    public boolean cameraOn = false;

    public Integer camId;

    public DetectionPipeline pipeline;


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
            ex3 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 173");
        } catch (Exception p_exception) {
            ex3 = null;
        }
        try { // Gyro
            gyro = hwMap.get(BNO055IMU.class, "g0");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
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

        try { // Intake wheel
            in = hwMap.get(DcMotor.class, "in");
            in.setDirection(DcMotor.Direction.FORWARD);
            in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            in.setPower(0);
            in.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            in = null;
        }

        try {
            shoot1 = hwMap.get(DcMotor.class, "shoot1");
            shoot1.setDirection(DcMotor.Direction.REVERSE);
            shoot1.setPower(0);
            shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ((DcMotorEx)shoot1).setVelocityPIDFCoefficients(30, 10, 3, 1);
        }
        catch(Exception p_exception) {
            shoot1 = null;
        }

        try {
            shoot2 = hwMap.get(DcMotor.class, "shoot2");
            shoot2.setDirection(DcMotor.Direction.REVERSE);
            shoot2.setPower(0);
            shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ((DcMotorEx)shoot2).setVelocityPIDFCoefficients(30, 10, 3, 1);
        }
        catch(Exception p_exception) {
            shoot2 = null;
        }

        try {
            wobblewheel = hwMap.get(DcMotor.class, "vw");
            wobblewheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wobblewheel.setPower(0);
        }
        catch(Exception p_exception) {
            wobblewheel = null;
        }

        try {
            flicker = hwMap.get(Servo.class, "f");
            flicker.setDirection(Servo.Direction.FORWARD);
            flicker.setPosition(Shooter.shootOut);
        }
        catch(Exception p_exception) {
            flicker = null;
        }

        try {
            wobble = hwMap.get(Servo.class, "w");
            wobble.setDirection(Servo.Direction.FORWARD);
            wobble.setPosition(1.0);
        }
        catch(Exception p_exception) {
            wobble = null;
        }

        try {
            camId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "wc1"), camId);
            pipeline = new DetectionPipeline();
        }
        catch(Exception e) {
            phoneCam = null;
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
     * Power the intake
     * @param power Proportion of max power for the intake
     */
    public void setIntakePower(double power) {
        if(in != null) {
            in.setPower(Range.clip(power, -Globals.MAX_IN_SPEED, Globals.MAX_IN_SPEED));
        }
    }

    /**
     * Reset omni encoders
     */
    public void resetOmnis() {
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Get current encoder reading of vertically aligned omni
     *
     * @return Encoder reading
     */
    public int getVOmniPos(RevBulkData rev) {
        if (rb != null) {
            return -rev.getMotorCurrentPosition(rb);
        }
        return 0;
    }

    public double getCurrent(DcMotor motor, CurrentUnit unit) {
        if(motor != null) {
            return ((DcMotorEx) motor).getCurrent(unit);
        }
        return 0;
    }

    /**
     * Get current encoder reading of horizontally aligned omni
     *
     * @return Encoder reading
     */
    public int getHOmniPos(RevBulkData rev) {
        if (rf != null) {
            return -rev.getMotorCurrentPosition(rf);
        }
        return 0;
    }

    public void cameraInit() {
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
    }

    public void cameraInit(OpenCvPipeline pipeline) {
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
    }

    public double getPower(DcMotor motor) {
        if(motor != null) {
            return motor.getPower();
        }
        return Double.NaN;
    }

    public void cameraOff() {
        phoneCam.closeCameraDevice();
    }
}