package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * org.firstinspires.ftc.teamcode.CactusRobot
 * This implements the hardware for the CactusRobot.  It is NOT an opmode, but is imported by al
 * of the opmodes implemented for this robot, so initialization, etc. will be consistent
 * <p>
 * init() instantiate this in your opmode
 * CactusRobot robot = new CactusRobot();   // Use a CactusRobot's hardware
 * robot.init(hardwareMap);
 * Then reference the hardware with things like
 * robot.leftBackDrive.setSpeed()
 */
public class CactusRobot {

    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor armRotate = null;
    public DcMotor fangs = null;
    public Servo gripper = null;
    public CRServo rightGripper = null;

    public ColorSensor forwardColor = null;
    public DistanceSensor forwardDistance = null;
    float hsvValues[] = {0F, 0F, 0F};       // hsvValues is an array that will hold the hue, saturation, and value information.
    final float values[] = hsvValues;       // values is a reference to the hsvValues array.

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    public ColorSensor downColor = null;
    public DistanceSensor downDistance = null;

    //    public int[] armPositions = {0, 870, 1460, 2160};
    public int[] armPositions = {50, 650, 1290, 1930};
    public double acceptableArmThumbPos = .20;
    public int armIndex = 0;
    public double armSpeed = .5;

    private static double gripClosePosition = .45;
    private static double gripOpenPosition = 0;
    private boolean isGripperOpen;

    HardwareMap hwMap = null;
    private Telemetry telemetry;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    /* don't have a blank constructor anymore, needs a reference to the telemetry object */
//    public CactusRobot() {
//    }

    public CactusRobot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Color Sensor
        forwardColor = hwMap.get(ColorSensor.class, "forwardColorDistance");
        forwardDistance = hwMap.get(DistanceSensor.class, "forwardColorDistance");

        downColor = hwMap.get(ColorSensor.class, "downColorDistance");
        downDistance = hwMap.get(DistanceSensor.class, "downColorDistance");

        // Define and Initialize Motors
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackDrive");
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fangs = hwMap.get(DcMotor.class, "fangMotor");
        fangs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRotate = hwMap.get(DcMotor.class, "armRotate");
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRotate.setDirection(DcMotor.Direction.REVERSE);
        armRotate.setPower(0);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set all motors to zero power
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        armRotate.setPower(0);
        fangs.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fangs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        gripper = hwMap.get(Servo.class, "gripper");

        openGripper();
        initializeArmPosition();
        moveArmToPosition(armIndex);
        closeGripper();
        this.waitFor(0.5);
        openGripper();
        this.waitFor(0.5);
    }

    public int getArmIndex() {
        return armIndex;
    }

    public void moveArmToPosition(int index) {
        int currentIndex = armIndex;
        if (index >= 0 && index < armPositions.length) {
            armIndex = index;
//            if (armIndex < 0) armIndex = 0;
//            if (armIndex > armPositions.length - 1) armIndex = armPositions.length - 1;
            if (index == 0 && (!isGripOpen()) && !(isBlockInRange())) { //if we're going to ground and we DON'T have a block already and the gripper is closed, open it
                this.openGripper();
                this.waitFor(.25);
            }
        }
        armRotate.setPower(0);
        armRotate.setTargetPosition(armPositions[armIndex]);
        armRotate.setPower(armSpeed);

    }

    public double getFrontDistance() {
        return Double.isNaN(this.forwardDistance.getDistance(DistanceUnit.CM)) ? 999 : this.forwardDistance.getDistance(DistanceUnit.CM);
    }

    public double getFrontColor() {
        Color.RGBToHSV((int) (forwardColor.red() * SCALE_FACTOR),
                (int) (forwardColor.green() * SCALE_FACTOR),
                (int) (forwardColor.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues[0];
    }

    public boolean isBlockColor() {
        return (getFrontDistance() < 15 && Math.abs(40 - this.getFrontColor()) <= 10);
    }

    public boolean isBlockInRange() {
        return (getFrontDistance() <= 6);
    }

    public void releaseBlock() {
        this.openGripper();
    }

    public void openGripper() {
        gripper.setPosition(gripOpenPosition);
        isGripperOpen = true;
    }

    public void closeGripper() {
        gripper.setPosition(gripClosePosition);
        isGripperOpen = false;
    }

    public boolean isGripOpen() {
        return isGripperOpen;
    }

    public void waitFor(double seconds) {
        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();

        while (timeoutTimer.time() < seconds) {
            telemetry.addData("Timer", timeoutTimer.time());
            telemetry.update();
        }
    }

    public void initializeArmPosition() {
        double prevPosition;
        boolean stillMoving = true;
        double sampleInterval = 0.05;
        double nextSampleTime = sampleInterval;
        double sample = 0;

        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();

        armRotate.setPower(0);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setPower(.2);
        this.waitFor(0.25);
        armRotate.setPower(0);
        this.waitFor(0.25);
        armRotate.setPower(-0.1);
        prevPosition = armRotate.getCurrentPosition();
        while (stillMoving && timeoutTimer.time() < 10) {
            telemetry.addData("armPosition", armRotate.getCurrentPosition());
            telemetry.addData("delta position", sample);
            telemetry.addData("Timer", timeoutTimer.time());
            telemetry.update();
            if (timeoutTimer.time() >= nextSampleTime) {
                sample = Math.abs(armRotate.getCurrentPosition() - prevPosition);
                stillMoving = (sample > 0 || nextSampleTime == sampleInterval);
                prevPosition = armRotate.getCurrentPosition();
                nextSampleTime = timeoutTimer.time() + sampleInterval;
            }
        }
        // back up a little bit to releive strain
        armRotate.setPower(.2);
        this.waitFor(0.25);
        armRotate.setPower(0);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.waitFor(0.25);

        armRotate.setTargetPosition(armPositions[0]);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetMotorEncoder(DcMotor theMotor) {
        DcMotor.RunMode oldMode = theMotor.getMode();

        theMotor.setPower(0);
        theMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        theMotor.setMode(oldMode);
    }
}
