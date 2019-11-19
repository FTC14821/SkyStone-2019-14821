package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(name = "FTC 14821 Teleop", group = "")
public class FTC14821_Teleop extends LinearOpMode {

    // create an instance of our robot hardware
    CactusRobot robot = new CactusRobot();
    RobotSettings settings = new RobotSettings();

    private static final double fastSpeed = 0.6;
    private static final double slowSpeed = 0.4;

    double gripClose;
    double gripOpen;
    double fangRelease;
    double fangGrab;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /**
     * Describe this function...
     */
    private void initialize() {
        gripClose = 0;
        gripOpen = 1;
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

                robot.leftDrive.setPower(-(driveSpeedScale * gamepad1.left_stick_y));
                robot.rightDrive.setPower(-(driveSpeedScale * gamepad1.right_stick_y));
                robot.armRotate.setPower(gamepad2.left_stick_y * 0.4);

                if (gamepad2.left_bumper == true) {
                    robot.leftGripper.setPower(1);
                    robot.rightGripper.setPower(1);
                } else if (gamepad2.right_bumper == true) {
                    robot.leftGripper.setPower(-1);
                    robot.rightGripper.setPower(-1);
                } else {
                    robot.leftGripper.setPower(0);
                    robot.rightGripper.setPower(0);
                }

                if (gamepad2.x == true) {
                    robot.fangs.setPower(0.1);
                } else if (gamepad2.y == true) {
                    robot.fangs.setPower(-0.1);
                } else {
                    robot.fangs.setPower(0);
                }

                telemetry.addData("leftPosition", robot.leftDrive.getCurrentPosition());
                telemetry.addData("rightPosition", robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    public double getHeading() {
        return 123.456;
    }
}
