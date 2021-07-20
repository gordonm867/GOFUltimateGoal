package org.firstinspires.ftc.teamcode.gofultimategoal.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
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
    public DcMotor in2;
    public DcMotor shoot1;
    public DcMotor shoot2;

    public ExpansionHubEx ex2;
    public ExpansionHubEx ex3;

    public RevBlinkinLedDriver led;
    public RevColorSensorV3 ringsensor;

    public Servo wobble;
    public Servo flicker;
    public Servo d1;
    public Servo d2;
    public Servo w1;
    public Servo c1;
    public Servo c2;

    public VoltageSensor battery;

    public boolean enabled;

    private static GOFHardware myInstance = null;

    public OpenCvCamera phoneCam;

    public boolean cameraOn = false;

    public Integer camId;

    public OpenCvPipeline pipeline;


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
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        try {
            ex2 = hwMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        } catch (Exception p_exception) {
            ex2 = null;
        }
        try {
            ex3 = hwMap.get(ExpansionHubEx.class, "Control Hub");
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
            telemetry.addData("Warning", "Gyro no work :(");
            telemetry.update();
            gyro = null;
        }

        try {
            battery = hwMap.get(VoltageSensor.class, "Control Hub");
        }
        catch (Exception p_exception) {
            battery = null;
            telemetry.addData("Warning", "Funny story: your autonomous is about to fail. :P");
            telemetry.update();
        }

        try {
            ringsensor = hwMap.get(RevColorSensorV3.class, "rs");
        }
        catch(Exception p_exception) {
            ringsensor = null;
            telemetry.addData("Warning", "Ring sensor bad :(");
            telemetry.update();
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
            in.setDirection(DcMotor.Direction.REVERSE);
            in.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            in.setPower(0);
            in.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception p_exception) {
            in = null;
        }

        try {
            shoot1 = hwMap.get(DcMotor.class, "shoot1");
            shoot1.setDirection(DcMotor.Direction.FORWARD);
            shoot1.setPower(0);
            shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch(Exception p_exception) {
            shoot1 = null;
        }

        try {
            shoot2 = hwMap.get(DcMotor.class, "shoot2");
            shoot2.setDirection(DcMotor.Direction.FORWARD);
            shoot2.setPower(0);
            shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch(Exception p_exception) {
            shoot2 = null;
        }

        try {
            in2 = hwMap.get(DcMotor.class, "vw");
            in2.setDirection(DcMotor.Direction.FORWARD);
            in2.setPower(0);
            in2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch(Exception p_exception) {
            in2 = null;
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
            wobble = hwMap.get(Servo.class, "w2");
            wobble.setDirection(Servo.Direction.FORWARD);
            wobble.setPosition(Wobble.closedpose);
        }
        catch(Exception p_exception) {
            wobble = null;
        }

        try {
            led = hwMap.get(RevBlinkinLedDriver.class, "led");
        }
        catch(Exception p_exception) {
            led = null;
        }

        try {
            d1 = hwMap.get(Servo.class, "d1");
            d1.setDirection(Servo.Direction.FORWARD);
            d1.setPosition(0.15);
        }
        catch(Exception p_exception) {
            d1 = null;
        }

        try {
            d2 = hwMap.get(Servo.class, "d2");
            d2.setDirection(Servo.Direction.FORWARD);
            d2.setPosition(0.65);
        }
        catch(Exception p_exception) {
            d2 = null;
        }

        try {
            w1 = hwMap.get(Servo.class, "w1");
            w1.setDirection(Servo.Direction.FORWARD);
            w1.setPosition(0.29);
        }
        catch(Exception p_exception) {
            w1 = null;
        }

        try {
            c1 = hwMap.get(Servo.class, "c1");
            c1.setDirection(Servo.Direction.FORWARD);
            c1.setPosition(0.5);
        }
        catch(Exception e) {}

        try {
            c2 = hwMap.get(Servo.class, "c2");
            c2.setDirection(Servo.Direction.FORWARD);
            c2.setPosition(0.6);
        }
        catch(Exception e) {}

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
     * @return Robot angle
     */
    public double getAngle() {
        return Functions.normalize(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + Globals.START_THETA);
    }

    /**
     * Get robot's angle
     *
     * @return Robot y angle
     */
    public double getYAngle() {
        return Functions.normalize(gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle + Globals.START_THETA);
    }

    /**
     * Get robot's angle
     *
     * @return Robot x angle
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
        if(in2 != null) {
            in2.setPower(Range.clip(power, -Globals.MAX_IN_SPEED, Globals.MAX_IN_SPEED));
        }
    }

    /**
     * Reset omni encoders
     */
    public void resetOmnis() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Get current encoder reading of vertically aligned omni
     *
     * @return Encoder reading
     */
    public int getVOmniPos(RevBulkData rev) {
        if (lf != null) {
            return -rev.getMotorCurrentPosition(lf);
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
        if (rb != null) {
            return -rev.getMotorCurrentPosition(rb);
        }
        return 0;
    }

    public void cameraInit() {
        Globals.BLUR_CONSTANT = 30;
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
    }

    public void cameraInit(OpenCvPipeline pipeline) {
        this.pipeline = pipeline;
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
    }

    public void changePipeline(OpenCvPipeline pipeline) {
        this.pipeline = pipeline;
        phoneCam.setPipeline(pipeline);
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
