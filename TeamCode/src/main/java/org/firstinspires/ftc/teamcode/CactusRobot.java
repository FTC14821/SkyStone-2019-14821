package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * org.firstinspires.ftc.teamcode.CactusRobot
 * This implements the hardware for the CactusRobot.  It is NOT an opmode, but is imported by al
 * of the opmodes implemented for this robot, so initialization, etc. will be consistent
 * <p>
 * init() instantiate this in your opmode
 * CactusRobot robot = new CactusRobot();   // Use a CactusRobot's hardware
 * robot.init(hardwareMap);        yeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeet
 * Then reference the hardware with things like
 * robot.leftBackDrive.setSpeed()
 */
public class CactusRobot {

    static final double MAX_VALID_DISTANCE = 20; //maximum valid distance from color/distance sensor
    static final double BLOCK_HUE = 45; // yellow block color
    static final int HUE_RANGE = 10; // how much the color can vary and still be in range

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
    private DataSampler forwardSampledDistance = null;

    float hsvValues[] = {0F, 0F, 0F};       // hsvValues is an array that will hold the hue, saturation, and value information.
    final float values[] = hsvValues;       // values is a reference to the hsvValues array.

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    public ColorSensor downColor = null;
    public DistanceSensor downDistance = null;

    //    public int[] armPositions = {0, 870, 1460, 2160};
    public int[] armPositions = {50, 675, 1290, 1930};
    public double acceptableArmThumbPos = .20;
    public int armIndex = 0;
    public double armSpeed = .5;

    private static double gripStartPosition = 0;
    private static double gripOpenPosition = 0.1;   // optimal is 0.1 and closed at 0.55
    private static double gripClosePosition = 0.55;  //
    private boolean isGripperOpen;

    public static int FANGS_UP_POSITION = 250;    // eg: Neverest 60
    public static int FANGS_DOWN_POSITION = 0;

    HardwareMap hwMap = null;
    private Telemetry telemetry;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */

    public CactusRobot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Color Sensor
        forwardColor = hwMap.get(ColorSensor.class, "forwardColorDistance");
        forwardDistance = hwMap.get(DistanceSensor.class, "forwardColorDistance");

        forwardSampledDistance = new DataSampler(3);

//        downColor = hwMap.get(ColorSensor.class, "downColorDistance");
//        downDistance = hwMap.get(DistanceSensor.class, "downColorDistance");

        // Define and Initialize Motors
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackDrive");
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fangs = hwMap.get(DcMotor.class, "fangMotor");
        fangs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fangs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armRotate = hwMap.get(DcMotor.class, "armRotate");
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setDirection(DcMotor.Direction.REVERSE);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fangs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        gripper = hwMap.get(Servo.class, "gripper");

        openGripper();
        initializeArmPosition();
        moveArmToPosition(armIndex);
        closeGripper();
        this.waitFor(0.5);
        initGripper();
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

    public double getForwardDistance() {
        double reading = this.forwardDistance.getDistance(DistanceUnit.CM);
        forwardSampledDistance.add((Double.isNaN(reading) || (reading > MAX_VALID_DISTANCE)) ? MAX_VALID_DISTANCE : reading / 2);
        return forwardSampledDistance.avg;
//        return (Double.isNaN(reading) || (reading > MAX_VALID_DISTANCE)) ? 999 : reading / 2;
    }

    public double getDownDistance() {
        return 999;
//        double reading = this.downDistance.getDistance(DistanceUnit.CM);
//        return Double.isNaN(reading) ? 999 : reading / 2;
    }

    public double getForwardColor() {
        Color.RGBToHSV((int) (forwardColor.red() * SCALE_FACTOR),
                (int) (forwardColor.green() * SCALE_FACTOR),
                (int) (forwardColor.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues[0];
    }

    /**
     * Check the block in front of us and return true if the color is in the range for the yellow bloock
     *
     * @return
     */
    public boolean isBlockColor() {
//        return (getForwardDistance() < 10 && Math.abs(40 - this.getForwardColor()) <= 10);
        return (Math.abs(BLOCK_HUE - this.getForwardColor()) <= HUE_RANGE);
    }

    /**
     * is the block in range to pick up?
     * @return true if close enough
     */
    public boolean isBlockInRange() {
        return (getForwardDistance() <= 6);
    }

    /**
     * releaseBlock will move back depending on the arm position and then release the block
     * TODO make this back up before opening gripper
     */
    public void releaseBlock() {
        this.openGripper();
    }

    /**
     * initGripper initializes the gripper by setting to a starting position (more than just open)
     * and also setting isGripperOpen to true
     */
    public void initGripper() {
        gripper.setPosition(gripStartPosition);
        isGripperOpen = true;
    }

    /**
     * openGripper will open the gripper and set isGripperOpen to true
     */
    public void openGripper() {
        gripper.setPosition(gripOpenPosition);
        isGripperOpen = true;
    }

    /**
     * closeGripper will close the gripper and set isGripperOpen to false
     */
    public void closeGripper() {
        gripper.setPosition(gripClosePosition);
        isGripperOpen = false;
    }

    /**
     * isGripOpen returns whether the gripper thinks it is open or closed
     * @return gripper open status (true=open)
     */
    public boolean isGripOpen() {
        return isGripperOpen;
    }

    /**
     * this waits for the specified number of seconds by just looping with telemetry update
     * @param seconds
     */
    public void waitFor(double seconds) {
        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();

        while (timeoutTimer.time() < seconds) {
//            telemetry.addData("Timer", timeoutTimer.time());
            telemetry.update();
        }
    }

    /**
     * initializeArmPosition() will attempt to move the arm to its lowest position
     * by moving it up a little, then down until the encoders aren't moving anymore
     * <p>The encoders keep reporting movement even when we're pretty close to done, so we check
     * at different intervals and see if it's "not moving much"</p>
     */
    public void initializeArmPosition() {
        double prevPosition;
        boolean stillMoving = true;
        double sampleInterval = 0.05;
        double nextSampleTime = sampleInterval;
        double sample = 0;

        ElapsedTime timeoutTimer = new ElapsedTime();

        armRotate.setPower(0);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // move the arm up a little so we will always have some down movement
        armRotate.setPower(0.3);
        this.waitFor(0.25);
        // stop and pause
        armRotate.setPower(0);
        this.waitFor(0.25);
        // now start moving it down very slowly
        armRotate.setPower(-0.1);

        prevPosition = armRotate.getCurrentPosition();
        timeoutTimer.reset();
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
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    /**
     * Reset any motor's encoder
     * This doesn't work correctly for some reason, so we just put the steps in everywhere
     * @deprecated
     * @param theMotor
     */
    public void resetMotorEncoder(DcMotor theMotor) {
        DcMotor.RunMode oldMode = theMotor.getMode();

        theMotor.setPower(0);
        theMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        theMotor.setMode(oldMode);
    }

    /**
     * Close the fangs to grab the foundation
     */
    public void closeFangs() {

        //true is down (biting)
        fangs.setTargetPosition(FANGS_DOWN_POSITION);
        fangs.setPower(-0.3); //DOWN
        fangs.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    /**
     * Open the fangs to let go of the foundation
     */
    public void openFangs() {
        //true is down (biting)
        fangs.setTargetPosition(FANGS_UP_POSITION);
        fangs.setPower(0.3); //UP
        fangs.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
