package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = " Teleop: FTC 14821", group = "any")
public class FTC14821_Teleop extends LinearGyroOpMode {

    // create an instance of our robot hardware
    CactusRobot robot = new CactusRobot(telemetry);
    RobotSettings settings = new RobotSettings();

    private static final double fastSpeed = 0.6;
    private static final double slowSpeed = 0.4;

    double fangRelease;
    double fangGrab;
    boolean isGripOpen;
    boolean bumperReleased;
    boolean armStickReleased = true;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /**
     * Describe this function...
     */
    private void initialize() {
        isGripOpen = true;
        bumperReleased = true;
        fangRelease = 0.65;
        fangGrab = 0;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//    parameters.loggingEnabled      = true;
//    parameters.loggingTag          = "IMU";
//    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        double driveSpeedScale;
        boolean savingSettings = false;
        int absoluteArmPosition = 0;

        robot.init(hardwareMap);
        /*
         * THIS IS IMPORTANT!!! IT LOADS THE SETINGS FROM A FILE
         * TODO Use the values from the settings object in the rest of our code
         */
        settings.readSettings();

        // Put initialization blocks here.
        initialize();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Hold "start", "back" and "y" to save current settings
                // This will just overwrite any file that's out there with the current settings
                // Which wil save the defaults if no file exists or the currently loaded settings if it did at startup
                if (gamepad1.start && gamepad1.back && gamepad1.y) {
                    if (!savingSettings) {
                        settings.writeSettings();
                        Log.d("CACTUS", "Saved Settings: ");
                        savingSettings = true;
                    }
                } else {
                    savingSettings = false;
                }
                // Put loop blocks here.
                if (gamepad1.left_bumper) {
                    // Drive SLOW mode (normal)
                    driveSpeedScale = settings.teleSlowSpeed;
                } else {
                    // Drive FAST mode
                    driveSpeedScale = settings.teleFastSpeed;
                }

                robot.leftBackDrive.setPower(-(driveSpeedScale * gamepad1.left_stick_y));
                robot.leftFrontDrive.setPower(-(driveSpeedScale * gamepad1.left_stick_y));
                robot.rightBackDrive.setPower(-(driveSpeedScale * gamepad1.right_stick_y));
                robot.rightFrontDrive.setPower(-(driveSpeedScale * gamepad1.right_stick_y));


                if (gamepad2.left_stick_y < -robot.acceptableArmThumbPos) {
                    if (armStickReleased) {
                        robot.moveArmToPosition(robot.getArmIndex() + 1);
                        armStickReleased = false;
                    }
                } else if (gamepad2.left_stick_y > robot.acceptableArmThumbPos) {
                    if (armStickReleased) {
                        robot.moveArmToPosition(robot.getArmIndex() - 1);
                        armStickReleased = false;
                    }
                } else {
                    armStickReleased = true;
                }

                if (gamepad2.b) {
                    absoluteArmPosition++;
                    robot.armRotate.setTargetPosition(absoluteArmPosition);
                    robot.armRotate.setPower(robot.armSpeed);
                }
                if (gamepad2.a) {
                    absoluteArmPosition--;
                    if (absoluteArmPosition < 0) absoluteArmPosition = 0;
                    robot.armRotate.setTargetPosition(absoluteArmPosition);
                    robot.armRotate.setPower(robot.armSpeed);
                }

                if (gamepad2.left_bumper && bumperReleased) {
                    if (isGripOpen) {
                        robot.closeGripper();
                    } else {
                        robot.openGripper();
                    }
                    isGripOpen = !isGripOpen;
                    bumperReleased = false;
                } else if (!gamepad2.left_bumper && !bumperReleased) {
                    bumperReleased = true;
                }

                if (gamepad2.y == true) {
                    robot.fangs.setTargetPosition(FANGS_UP_POSITION);
                    robot.fangs.setPower(0.3); //UP
                    robot.fangs.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.x == true) {
                    robot.fangs.setTargetPosition(FANGS_DOWN_POSITION);
                    robot.fangs.setPower(-0.2); //DOWN
                    robot.fangs.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
//                    robot.fangs.setPower(0);
                }

                telemetry.addData("armPosition", robot.armRotate.getCurrentPosition());
                telemetry.addData("armTargetPosition", robot.armPositions[robot.armIndex]);
                telemetry.addData("armIndex", robot.getArmIndex());
                telemetry.addData("leftPosition", robot.leftBackDrive.getCurrentPosition());
                telemetry.addData("rightPosition", robot.rightBackDrive.getCurrentPosition());
                telemetry.addData("gripperPosition", robot.gripper.getPosition());
                telemetry.addData("fangPosition", robot.fangs.getCurrentPosition());
                telemetry.update();
            }
        }
    }

}
