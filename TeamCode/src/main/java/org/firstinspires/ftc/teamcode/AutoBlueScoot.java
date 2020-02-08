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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto BLUE Scoot", group = "Blue")
public class AutoBlueScoot extends LinearGyroOpMode {

    @Override
    public void runOpMode() {

        double heading = 0;
        double blockPosition;

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
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

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        //while (!isStopRequested() && gyro.isCalibrating())  {
        // sleep(50);
        //idle();
        //}

        imu.startAccelerationIntegration(null, null, 1000);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Ready.");    //
            telemetry.update();
        }

        double HOLD=0.4; //standard hold time
        double longDistance = 38;
        double blockWidth = 8.0;

        heading = 0; //start at this heading
        robot.openGripper();
        gyroDrive(AUTO_DRIVE_SLOW, 32.0, heading, 10, 6);
        blockPosition = 0;
        while (!isSkystone() && blockPosition < 2) {
            scoot("right");
            blockPosition++;
        }
        robot.closeGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, 1);
        robot.moveArmToPosition(1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, -8.0, heading,.5);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // turn right and drive to the other side
        heading = 90;    //LEFT turn postive, RIGHT turn negative
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_FAST, longDistance + blockPosition * blockWidth, heading, 3);
        // spit the block out by running grippers for 0.75 seconds
        robot.moveArmToPosition(0);
        robot.openGripper();

        // now go  back and get next skystone
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);
        blockPosition = 3;
        gyroDrive(AUTO_DRIVE_FAST, -(longDistance + blockPosition * blockWidth), heading, 6);
        heading = 0;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        //drive up to the next stone and start looking
        gyroDrive(AUTO_DRIVE_SLOW, 8.0, heading,.5, 6);
        while (!isSkystone() && blockPosition < 4) {
            scoot("right");
            blockPosition++;
        }
        robot.closeGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, 1);
        robot.moveArmToPosition(1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, -8.0, heading,.5);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // turn right and drive to the other side
        heading = 90;    //LEFT turn postive, RIGHT turn negative
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_FAST, longDistance + blockPosition * blockWidth, heading, 3);
        // spit the block out by running grippers for 0.75 seconds
        robot.moveArmToPosition(0);
        robot.openGripper();

        //Now go to the line
        // spit the block out by running grippers for 0.75 seconds
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        gyroDrive(AUTO_DRIVE_SLOW, -16.0, heading);
        gyroHold(AUTO_DRIVE_SLOW, heading, 1);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
