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

@Autonomous(name = "TEST Auto Drive", group = "TEST")
public class AutoDriveTest extends LinearAutoGyroOpMode {

    @Override
    public void runOpMode() {

        initOpMode();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Ready.");    //
            telemetry.update();
        }

        double heading = 0;
        heading=0; //start at this heading
//        gyroDrive(DRIVE_SPEED, 36, heading, 10, 7);
//        gyroHold(DRIVE_SPEED, heading, 1);
//        scootAngle("left",90);
        scoot("left");
        gyroHold(DRIVE_SPEED, heading, 1);
        scoot("left");
//        scootAngle("left",75);
        gyroHold(DRIVE_SPEED, heading, 1);
        scoot("left");
//        scootAngle("left",45);
        gyroHold(DRIVE_SPEED, heading, 1);
        scoot("left");
//        scootAngle("left",30);
        gyroHold(DRIVE_SPEED, heading, 1);

        //
        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
