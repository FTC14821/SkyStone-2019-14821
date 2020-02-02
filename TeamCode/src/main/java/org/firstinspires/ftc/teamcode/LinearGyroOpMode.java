/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is Team 14821 (Cactus Intelligence Agency) extension of LinearOpMode class that adds
 * functionality for GyroDrive, GyroTurn, and GyroHold which use the built-in IMU to drive
 * to/on a specific heading.
 * <p>
 * LinearGyroOpMode class implements
 * CactusRobot robot
 * <p>
 * <p>
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the robot is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 * <p>
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Autonomous(name = "Cactus: Auto BLUE Blockside", group = "BLUE")
public abstract class LinearGyroOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    public CactusRobot robot = new CactusRobot();   // Use a Pushbot's hardware

    //    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    public BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters;
    public double COUNTS_PER_MOTOR_REV = 536;    // eg: TETRIX Motor Encoder
//    public double COUNTS_PER_MOTOR_REV = 753.2;

    public double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference

    public double COUNTS_PER_INCH = 0; //(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public double DRIVE_SPEED = 0.3;     // Nominal speed for better accuracy.
    public double TURN_SPEED = 0.2;     // Nominal half speed for better accuracy.

    public double HEADING_THRESHOLD = 4;      // As tight as we can make it with an integer gyro
    public double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    public double P_DRIVE_COEFF = 0.1;     // Larger is more responsive, but also less stable

    public int FANGS_UP_POSITION = 275;    // eg: Neverest 60
    public int FANGS_DOWN_POSITION = 0;

    public double TEST = 0;

    /**
     * Constructor for LinearGyroOpMode sets initial COUNTS_PER_INCH
     */
    public LinearGyroOpMode() {
        this.TEST = Math.random() * 100;
        this.calcCountsPerInch();
    }

    /**
     * Constructor for LinearGyroOpmode with defaults for key values
     *
     * @param COUNTS_PER_MOTOR_REV  Number of encoder counts per drive wheel revolution
     * @param DRIVE_GEAR_REDUCTION  Gear reduction (<1.0 if geared up)
     * @param WHEEL_DIAMETER_INCHES Drive wheel diameter in inches
     */
    public LinearGyroOpMode(double COUNTS_PER_MOTOR_REV, double DRIVE_GEAR_REDUCTION, double WHEEL_DIAMETER_INCHES) {
        this.COUNTS_PER_MOTOR_REV = COUNTS_PER_MOTOR_REV;
        this.DRIVE_GEAR_REDUCTION = DRIVE_GEAR_REDUCTION;
        this.WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_INCHES;
        this.calcCountsPerInch();
    }

    public void setCOUNTS_PER_MOTOR_REV(double COUNTS_PER_MOTOR_REV) {
        this.COUNTS_PER_MOTOR_REV = COUNTS_PER_MOTOR_REV;
        this.calcCountsPerInch();
    }

    public void setDRIVE_GEAR_REDUCTION(double DRIVE_GEAR_REDUCTION) {
        this.DRIVE_GEAR_REDUCTION = DRIVE_GEAR_REDUCTION;
        this.calcCountsPerInch();
    }

    public void setWHEEL_DIAMETER_INCHES(double WHEEL_DIAMETER_INCHES) {
        this.WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_INCHES;
        this.calcCountsPerInch();
    }

    private double calcCountsPerInch() {
        // recalculate COUNTS_PER_INCH but protect against zero value for wheel diameter
        this.COUNTS_PER_INCH = (this.WHEEL_DIAMETER_INCHES != 0) ?
                (this.COUNTS_PER_MOTOR_REV * this.DRIVE_GEAR_REDUCTION) / (this.WHEEL_DIAMETER_INCHES * 3.1415) : 0;
        return this.COUNTS_PER_INCH;
    }


    public void gyroDrive(double speed,
                          double distance,
                          double angle) {
        gyroDrive(speed, distance, angle, 15, 0);
    }

    public void gyroDrive(double speed,
                          double distance,
                          double angle,
                          double timeout) {
        gyroDrive(speed, distance, angle, timeout, 0);
    }

        /**
         * Method to drive on a fixed compass bearing (angle), based on encoder counts.
         * Move will stop if either of these conditions occur:
         * 1) Move gets to the desired position
         * 2) Driver stops the opmode running.
         *
         * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
         * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
         * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
         *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
         *                 If a relative angle is required, add/subtract from current heading.
         * @param timeout  Number of seconds (can be fractional) before timing out, whether we get there or not
         */
    public void gyroDrive(double speed,
                          double distance,
                          double angle,
                          double timeout,
                          double frontDistance) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftBackDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftBackDrive.setTargetPosition(newLeftTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftTarget);
            robot.rightBackDrive.setTargetPosition(newRightTarget);
            robot.rightFrontDrive.setTargetPosition(newRightTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftBackDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);

            ElapsedTime timeoutTimer = new ElapsedTime();
            timeoutTimer.reset();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy()) && robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() &&
                    (Double.isNaN(robot.forwardDistance.getDistance(DistanceUnit.CM)) ||
                            robot.forwardDistance.getDistance(DistanceUnit.CM) > frontDistance) &&
                    (timeoutTimer.time() < timeout)) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftBackDrive.setPower(leftSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightBackDrive.setPower(rightSpeed);
                robot.rightFrontDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftBackDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftBackDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftBackDrive.setPower(leftSpeed);
        robot.rightBackDrive.setPower(rightSpeed);
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.rightFrontDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        Orientation angles;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - imu.getIntegratedZValue();
        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
