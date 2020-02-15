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

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

    enum Alliance {
        BLUE(0), RED(1);
        public final byte bVal;

        Alliance(int i) {
            bVal = (byte) i;
        }
    }

    /* Declare OpMode members. */
    public CactusRobot robot = new CactusRobot(telemetry);   // Use a Pushbot's hardware

    //    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    public BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters;

    /* GoBilda Motor Encoder counts https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motors/
        223RPM : 753.2
        312RPM : 537.6
     */
//    public double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    public double COUNTS_PER_MOTOR_REV = 753.2;

    public double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference

    public double COUNTS_PER_INCH = 0; //(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public double DRIVE_SPEED = 0.4;     // Nominal speed for better accuracy.
    public double AUTO_DRIVE_FAST = .8;     // Nominal speed for better accuracy.
    public double AUTO_DRIVE_SLOW = 0.6;     // Nominal speed for better accuracy.
    public double TURN_SPEED = 0.6;     // Nominal half speed for better accuracy.
    public double AUTO_TURN_SPEED = 0.6;     // Nominal half speed for better accuracy.

    public double STRAIGHT_THRESHOLD = COUNTS_PER_MOTOR_REV * 0.03; // 1% of one rotation
    public double HEADING_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    public double P_TURN_COEFF = 0.02;     // Larger is more responsive, but also less stable
    public double P_DRIVE_COEFF = 0.02;     // Larger is more responsive, but also less stable

    public int FANGS_UP_POSITION = 275;    // eg: Neverest 60
    public int FANGS_DOWN_POSITION = 0;

    static public double HOLD = 0.4; //standard hold time to wait for things to finish
    //TODO - replace all the random hold times that are ~0.5 sec to use HOLD so we can tune as fast as possible

    public double lastHeading; //saves the most recent heading we drove or turned to

    /**
     * Constructor for LinearGyroOpMode sets initial COUNTS_PER_INCH
     */
    public LinearGyroOpMode() {
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

    public void initOpMode() {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        Log.d("Constructor", "starting robot.init");
        robot.init(hardwareMap);

        telemetry.addData(">", "Calibrating IMU");    //
        telemetry.update();

        Log.d("Constructor", "starting IMU");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);

        imu.startAccelerationIntegration(null, null, 1000);

        telemetry.addData(">", "OpMode Initialized.");    //
        telemetry.update();
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
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int moveCounts;
        double closeEnoughLevel;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        ElapsedTime timeoutTimer = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);

            newLeftTarget = robot.leftBackDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftBackDrive.setTargetPosition(newLeftTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);

            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.1, 1.0);
            robot.leftBackDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);

            timeoutTimer.reset();

            boolean isTimeout = false;
            boolean isFrontDistance = false;
            boolean isBusy = true;
            boolean isCloseEnough = false;

            Log.d("gyrodrive", "Distance=" + distance + ", Angle=" + angle + ", Speed=" + speed + ", Timeout=" + timeout + ", FrontDistance=" + frontDistance);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && !isCloseEnough && isBusy && !isTimeout && !isFrontDistance) {
//                    (robot.leftBackDrive.isBusy() || robot.leftFrontDrive.isBusy()) &&
//                    (robot.rightBackDrive.isBusy() || robot.rightFrontDrive.isBusy()) &&
//                    (robot.getForwardDistance() > forwardDistance) &&
//                    (timeoutTimer.time() < timeout)) {

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

                leftSpeed = Range.clip(Math.abs(leftSpeed), 0.05, 1.0);
                rightSpeed = Range.clip(Math.abs(rightSpeed), 0.05, 1.0);

                Log.d("gyrodrive", "leftSpeed");

                robot.leftBackDrive.setPower(leftSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightBackDrive.setPower(rightSpeed);
                robot.rightFrontDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Front Target/Actual", "%7d/%7d\t%7d/%7d", newLeftFrontTarget, robot.leftFrontDrive.getCurrentPosition(), newRightFrontTarget, robot.rightFrontDrive.getCurrentPosition());
                telemetry.addData("Back Target/Actual", "%7d/%7d\t%7d/%7d", newLeftTarget, robot.leftBackDrive.getCurrentPosition(), newRightTarget, robot.rightBackDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();

                // if the average position of all four drive wheels is within 99% of the target, we're done
                isCloseEnough = (Math.abs((double) robot.leftBackDrive.getCurrentPosition() - (double) newLeftTarget)
                        + Math.abs((double) robot.rightBackDrive.getCurrentPosition() - (double) newRightTarget)
                        + Math.abs((double) robot.leftFrontDrive.getCurrentPosition() - (double) newLeftFrontTarget)
                        + Math.abs((double) robot.rightFrontDrive.getCurrentPosition() - (double) newRightFrontTarget)) / 4 < STRAIGHT_THRESHOLD;

                isBusy = (robot.leftBackDrive.isBusy() || robot.leftFrontDrive.isBusy()) || (robot.rightBackDrive.isBusy() || robot.rightFrontDrive.isBusy());
                isFrontDistance = (robot.getForwardDistance() <= frontDistance);
                isTimeout = (timeoutTimer.time() >= timeout);
            }
            Log.d("gyrodrive", "isBusy " + isBusy);
            Log.d("gyrodrive", "isCloseEnough " + isCloseEnough);
            Log.d("gyrodrive", "isFrontDistance " + isFrontDistance);
            Log.d("gyrodrive", "isTimeout " + isTimeout);

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

            lastHeading = angle;
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
        this.gyroTurn(speed, angle, 10);
    }

    public void gyroTurn(double speed, double angle, double timeout) {
        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && (timeoutTimer.time() < timeout)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        lastHeading = angle;
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
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        lastHeading = angle;
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
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.leftBackDrive.setPower(leftSpeed);
        robot.rightFrontDrive.setPower(rightSpeed);
        robot.rightBackDrive.setPower(rightSpeed);

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

    public boolean isSkystone() {
        return (robot.getForwardColor() > 100 && robot.getForwardDistance() < 15);
    }

    public void scootAngle(String direction, double angle) {
        double turnAngle = (Math.abs(angle) <= 90 && angle != 0) ? angle : 90; //default to 90
        double minBack = 5; // minimum distance backwards to drive
        double blockSize = 8; // width of the blocks

        // calculate the distance backwards (dY) and alongthe hypotenuse (dH) for the angle we want to turn
        // and avoid dividing by zero or infinity
        double dY = (turnAngle != 90) ? Math.abs(blockSize / Math.tan(Math.toRadians(turnAngle))) : 0; //how far back to we have to go to get our mark
        double dH = Math.abs(blockSize / Math.sin(Math.toRadians(turnAngle))); //already corrected for turnAngle > 0 so it's safe

        if (direction.toLowerCase() == "right") {
            turnAngle *= -1;
        }

        double originalHeading = lastHeading;
        double heading = originalHeading;

        // back up minBack + calculated distance
        gyroDrive(AUTO_DRIVE_SLOW, -(minBack + dY), heading);

        // turn to our designated angle
        heading += turnAngle;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);

        // drive on that heading for the hypotenuse
        gyroDrive(AUTO_DRIVE_SLOW, dH, heading, 2);

        // turn back to original heading
        heading = originalHeading;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);

        // now drive forward the minBack distance
        gyroDrive(AUTO_DRIVE_SLOW, minBack, heading, 2, 6);
    }


    public void scoot(String direction) {

        // can just replace with this really
//        scootAngle(direction, 90);

        double turnAngle = 90;
        if (direction.toLowerCase() == "right") {
            turnAngle *= -1;
        }
        double originalHeading = lastHeading;
        double heading = originalHeading;

        gyroDrive(AUTO_DRIVE_SLOW, -5, heading);
        heading += turnAngle;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroDrive(AUTO_DRIVE_SLOW, 8.0, heading, 1);
        heading = originalHeading;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, 5, heading, 2, 6);
    }

}

