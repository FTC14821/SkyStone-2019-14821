package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "14821 Main Teleop", group = "ANY")
public class FTC14821_Teleop extends LinearGyroOpMode {

    RobotSettings settings = new RobotSettings();

    boolean bumperReleased;
    boolean armStickReleased = true;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        double driveSpeedScale;
        boolean savingSettings = false;
        int absoluteArmPosition = 0;

        initOpMode(false);

        /*
         * THIS IS IMPORTANT!!! IT LOADS THE SETINGS FROM A FILE
         * TODO Use the values from the settings object in the rest of our code
         */

        // don't read the settings file, just take the defaults
//        settings.readSettings();

        bumperReleased = true;

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

                // negative moves UP
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

                if (robot.isGripOpen() && bumperReleased && (robot.armIndex == 0) && robot.isBlockInRange() && robot.isBlockColor()) {
//                    robot.closeGripper();
                }

                bumperReleased = bumperReleased || (!gamepad2.left_bumper);
                if (gamepad2.left_bumper && bumperReleased) {
                    if (robot.isGripOpen()) {
                        robot.closeGripper();
                    } else {
                        robot.openGripper();
                    }
                    bumperReleased = false;
                }

                if (gamepad2.y == true) {
                    robot.openFangs();
                } else if (gamepad2.x == true) {
                    robot.closeFangs();
                } else {
//                    robot.fangs.setPower(0);
                }

                telemetry.addData("front (left) (right)", "(%,d) (%,d)", robot.leftFrontDrive.getCurrentPosition(), robot.rightFrontDrive.getCurrentPosition());
                telemetry.addData("back  (left) (right)","(%,d) (%,d)", robot.leftBackDrive.getCurrentPosition(), robot.rightBackDrive.getCurrentPosition());
                telemetry.addData("Arm Index : Target : Position", "%,d : %,d : %,d",robot.getArmIndex(),robot.armPositions[robot.armIndex],robot.armRotate.getCurrentPosition());
                telemetry.addData("Front Color/Distance Distance : Color", "%,.2f : %,.2f", robot.getForwardDistance(),robot.getForwardColor());
                telemetry.addData("gripperPosition", robot.gripper.getPosition());
                telemetry.addData("fangPosition", robot.fangs.getCurrentPosition());
                telemetry.update();
            }
        }
    }

}
