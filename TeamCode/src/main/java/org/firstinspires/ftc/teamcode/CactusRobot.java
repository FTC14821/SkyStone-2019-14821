package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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

    //    public int[] armPositions = {0, 870, 1460, 2160};
    public int[] armPositions = {0, 650, 1290, 1930};
    public double acceptableArmThumbPos = .20;
    public int armIndex = 0;
    public double armSpeed = .5;

    public double gripClose = .45;
    public double gripOpen = 0;

    HardwareMap hwMap = null;
    private Telemetry telemetry;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public CactusRobot() {
    }

    public CactusRobot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Color Sensor
        forwardColor = hwMap.get(ColorSensor.class, "forwardColorDistance");
        forwardDistance = hwMap.get(DistanceSensor.class, "forwardColorDistance");

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

        armRotate = hwMap.get(DcMotor.class, "armRotate");
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setDirection(DcMotor.Direction.REVERSE);
        armRotate.setPower(0);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveArmToPosition(armIndex);

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
//        this.closeGripper();
        openGripper();
//        this.waitFor(1);
        initializeArmPosition();
        closeGripper();
        this.waitFor(1);
    }

    public int getArmIndex() {
        return armIndex;
    }

    public void moveArmToPosition(int index) {
        armIndex = index;
        if (armIndex < 0) armIndex = 0;
        if (armIndex > armPositions.length - 1) armIndex = armPositions.length - 1;
        armRotate.setPower(0);
        armRotate.setTargetPosition(armPositions[armIndex]);
        armRotate.setPower(armSpeed);
    }

    public void releaseBlock() {
        this.openGripper();
    }

    public void openGripper() {
        gripper.setPosition(gripOpen);
    }

    public void closeGripper() {
        gripper.setPosition(gripClose);
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
        double nextSampleTime = 0.1;

        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();

        armRotate.setPower(0);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setPower(-0.1);
        prevPosition = armRotate.getCurrentPosition();
        while (stillMoving && timeoutTimer.time() < 7) {
            telemetry.addData("armPosition", armRotate.getCurrentPosition());
            telemetry.addData("Timer", timeoutTimer.time());
            telemetry.update();
            if (timeoutTimer.time() >= nextSampleTime) {
                stillMoving = (Math.abs(armRotate.getCurrentPosition() - prevPosition) > 0);
                prevPosition = armRotate.getCurrentPosition();
                nextSampleTime = timeoutTimer.time() + 0.1;
            }
        }
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armRotate.setTargetPosition(0);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
