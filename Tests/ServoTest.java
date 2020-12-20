package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@Disabled
public class ServoTest {
    Servo servo;
    String SERVO_NAME = "";

    double increment = 0.1;
    double pos = 0;
    ArrayList<Double> myposes = new ArrayList<>();

    boolean xpressed = false;
    boolean apressed = false;
    boolean bpressed = false;
    boolean dlpressed = false;
    boolean drpressed = false;
    boolean dupressed = false;
    boolean ddpressed = false;
    boolean lbumper = false;
    boolean rbumper = false;
    boolean rtrigger = false;
    boolean ltrigger = false;
    boolean ypressed = false;

    boolean freeze = false;

    int save = 0;

    double last = 0;

    HardwareMap hardwareMap = null;
    Telemetry telemetry;

    /**
     * For an initialized servo
     * @param servo The servo
     * @param telemetry The Telemetry stream from the OpMode ("telemetry")
     */
    public ServoTest(Servo servo, Telemetry telemetry) {
        this.servo = servo;
        this.telemetry = telemetry;
    }

    /**
     * For a servo you have NOT initialized yet
     * @param servo The servo
     * @param hardwareMap The HardwareMap object from your OpMode ("hardwareMap")
     * @param servoName The name of the servo on the configuration on your phones
     * @param telemetry The Telemetry stream from the OpMode ("telemetry")
     */
    public ServoTest(Servo servo, HardwareMap hardwareMap, String servoName, Telemetry telemetry) {
        this.servo = servo;
        this.hardwareMap = hardwareMap;
        SERVO_NAME = servoName;
        this.telemetry = telemetry;
        try {
            servo = hardwareMap.get(Servo.class, SERVO_NAME);
            pos = servo.getPosition();
        }
        catch(Exception e) {
            telemetry.addData("Oh no", "Your servo was not found!  Are you sure it is called \"" + SERVO_NAME + "\" on your configuration on the phone?");
            telemetry.addData("Note", "Remember that these names are case sensitive!");
            telemetry.update();
        }
    }

    public void run(Gamepad gamepad1) {
        if(gamepad1.x && !xpressed) { // Save
            xpressed = true;
            myposes.add(pos);
        }
        if(!gamepad1.x) {
            xpressed = false;
        }
        if(gamepad1.a && !apressed) { // Multiply increment by 10
            apressed = true;
            increment *= 10;
        }
        if(!gamepad1.a) {
            apressed = false;
        }
        if(gamepad1.b && !bpressed) { // Divide increment by 10
            bpressed = true;
            increment /= 10;
        }
        if(!gamepad1.b) {
            bpressed = false;
        }
        if(gamepad1.dpad_left && !dlpressed) { // Move backwards through your saved position list
            dlpressed = true;
            if(myposes.size() == 0 || save == 0) {
                save = 0;
            }
            else {
                save = (save - 1) % myposes.size();
            }
        }
        if(!gamepad1.dpad_left) {
            dlpressed = false;
        }
        if(gamepad1.dpad_right && !drpressed) { // Move forwards through your saved position list
            drpressed = true;
            if(myposes.size() == 0) {
                save = 0;
            }
            else {
                save = (save + 1) % myposes.size();
            }
        }
        if(!gamepad1.dpad_right) {
            drpressed = false;
        }
        if(gamepad1.dpad_up && !dupressed && myposes.size() > 0) { // Go to the selected saved position
            dupressed = true;
            pos = myposes.get(save);
        }
        if(!gamepad1.dpad_up) {
            dupressed = false;
        }
        if(gamepad1.dpad_down && !ddpressed && myposes.size() > 0) { // Remove the selected save position
            ddpressed = true;
            myposes.remove(save);
            save = 0;
        }
        if(!gamepad1.dpad_down) {
            ddpressed = false;
        }
        if(gamepad1.left_bumper && !lbumper) { // Decrease servo position by increment
            lbumper = true;
            pos -= increment;
            pos = Math.max(0, pos);
        }
        if(!gamepad1.left_bumper) {
            lbumper = false;
        }
        if(gamepad1.right_bumper && !rbumper) { // Increase servo position by increment
            rbumper = true;
            pos += increment;
            pos = Math.min(1, pos);
        }
        if(!gamepad1.right_bumper) {
            rbumper = false;
        }
        if(gamepad1.left_trigger > 0.1 && !ltrigger) { // Reverse servo
            ltrigger = true;
            servo.setDirection(servo.getDirection() == Servo.Direction.FORWARD ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        }
        if(!(gamepad1.left_trigger > 0.1)) {
            ltrigger = false;
        }
        if(gamepad1.right_trigger > 0.1 && !rtrigger) { // Freeze/unfreeze servo temporarily
            rtrigger = true;
            freeze = !freeze;
        }
        if(!(gamepad1.right_trigger > 0.1)) {
            rtrigger = false;
        }
        if(gamepad1.y && !ypressed) { // Delete all saved poses
            ypressed = true;
            myposes.clear();
        }
        if(!gamepad1.y) {
            ypressed = false;
        }

        if(!freeze) {
            servo.setPosition(pos);
            last = pos;
            telemetry.addData("Position", pos);
        }
        else {
            telemetry.addData("Target position", pos);
            telemetry.addData("Frozen at", last);
        }
        telemetry.addData("Increment", increment);
        telemetry.addData("\n", "\n");
        StringBuilder saved = new StringBuilder();
        for(Double pose : myposes) {
            saved.append(pose).append(", ");
        }
        saved.append("516 :P"); // I mean, you wouldn't want the hanging comma....
        telemetry.addData("YOUR SAVED POSES", saved.toString());
        telemetry.addData("\n", "\n");
        if(myposes.size() > 0) {
            telemetry.addData("Clicking DPAD_UP", "will GO TO position index " + save + " (" + myposes.get(save) + ") ");
            telemetry.addData("Clicking DPAD_DOWN", "will DELETE position index " + save + " (" + myposes.get(save) + ") ");
            telemetry.addData("Clicking DPAD_LEFT", "will move that index DOWN");
            telemetry.addData("Clicking DPAD_RIGHT", "will move that index UP");
        }
        telemetry.addData("Clicking right_bumper", "will INCREASE the servo position by " + increment);
        telemetry.addData("Clicking left_bumper", "will DECREASE the servo position by " + increment);
        telemetry.addData("Clicking right_trigger", "will " + (freeze ? "UNFREEZE" : "FREEZE") + " the servo in place temporarily");
        telemetry.addData("Clicking left_trigger", "will set the servo direction to " + (servo.getDirection() == Servo.Direction.REVERSE ? "FORWARD" : "REVERSE"));
        telemetry.addData("Clicking a", "will INCREASE the increment to " + increment * 10.0);
        telemetry.addData("Clicking b", "will DECREASE the increment to " + increment / 10.0);
        telemetry.addData("Clicking x", "will SAVE your current position");
        telemetry.addData("Clicking y", "will DELETE all saved values");
        telemetry.update();
    }
}
