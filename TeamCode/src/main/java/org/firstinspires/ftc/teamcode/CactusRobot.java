package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    HardwareMap hwMap = null;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public CactusRobot() {
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Color Sensor
        forwardColor = hwMap.get(ColorSensor.class, "forwardColorDistance");
        forwardDistance = hwMap.get(DistanceSensor.class, "forwardColorDistance");

        // Define and Initialize Motors
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackDrive");
        leftFrontDrive = hwMap.get(DcMotor.class,"leftFrontDrive");
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
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fangs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        gripper = hwMap.get(Servo.class, "gripper");
//        rightGripper = hwMap.get(CRServo.class, "rightGripper");
//        rightGripper.setDirection(DcMotorSimple.Direction.REVERSE);
        //gripper.setPower(0);
//        rightGripper.setPower(0);

    }
}
