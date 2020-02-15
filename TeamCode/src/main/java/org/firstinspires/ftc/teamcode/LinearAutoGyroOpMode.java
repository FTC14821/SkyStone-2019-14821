package org.firstinspires.ftc.teamcode;

public abstract class LinearAutoGyroOpMode extends LinearGyroOpMode {

    public static int WALL_TO_BLOCK_DISTANCE = 32;
    public static int BLOCK_STOP_DISTANCE = 7;

    public void autoTwoSkystones(Alliance color) {
        double heading = 0;
        double blockPosition;
        double longDistance = 38;
        double blockWidth = 8.0;

        String scootDirection = (color == Alliance.BLUE) ? "right" : "left";
        int turnDirection = (color == Alliance.BLUE) ? 1 : -1;

        heading = 0; //start at this heading
        robot.openGripper();
        gyroDrive(AUTO_DRIVE_SLOW, 32.0, heading, 10, 6);
        blockPosition = 0;
        while (!isSkystone() && blockPosition < 2) {
            scoot(scootDirection);
            blockPosition++;
        }
        robot.closeGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, 1);
        robot.moveArmToPosition(1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, -8.0, heading,1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // turn right and drive to the other side
        heading = 90 * turnDirection;    //LEFT turn postive, RIGHT turn negative
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_FAST, longDistance + blockPosition * blockWidth, heading, 10);
        // spit the block out by running grippers for 0.75 seconds
        robot.openGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // now go  back and get next skystone
        blockPosition += 3;
        gyroDrive(AUTO_DRIVE_FAST, -(longDistance + blockPosition * blockWidth), heading, 10);
        robot.moveArmToPosition(0); //lower arm
        heading = 0;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);

        //drive up to the next stone and start looking
        gyroDrive(AUTO_DRIVE_SLOW, 8.0, heading,2, 6);
        robot.closeGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, 1); //wait for gripper to close
        robot.moveArmToPosition(1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, -8.0, heading,2);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // turn right and drive to the other side
        heading = 90 * turnDirection;    //LEFT turn postive, RIGHT turn negative
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_FAST, longDistance + blockPosition * blockWidth, heading, 10);
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

    public void autoGetFirstBlock(Alliance color) {

        int turnDirection = (color == Alliance.BLUE) ? 1 : -1;

        double heading = 0;

        heading = 0; //start at this heading
        robot.openGripper();
        gyroDrive(DRIVE_SPEED, 32.0, heading, 10, 6);
        robot.closeGripper();
        gyroHold(DRIVE_SPEED, heading, 1);
        robot.moveArmToPosition(1);
        gyroHold(DRIVE_SPEED, heading, HOLD);
        gyroDrive(DRIVE_SPEED, -2.0, heading,.5);
        gyroHold(DRIVE_SPEED, heading, HOLD);

        // turn right and drive to the other side
        heading = 90 * turnDirection;    //LEFT turn postive, RIGHT turn negative
        gyroTurn(TURN_SPEED, heading);
        gyroHold(TURN_SPEED, heading, HOLD);
        gyroDrive(DRIVE_SPEED*2, 40.0, heading, 3);

        // spit the block out by running grippers for 0.75 seconds
        robot.moveArmToPosition(0);
        robot.openGripper();
        gyroHold(DRIVE_SPEED, heading, HOLD);

        gyroDrive(DRIVE_SPEED, -16.0, heading);
        gyroHold(DRIVE_SPEED, heading, 1);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    // Drive forward to the bridge, turn, and park on line
    // Starting on the FOUNDATION side, so BLUE turns RIGHT (-1)
    //      RED turns LEFT (+1)
    public void autoParkOnLineAtBridge(Alliance color) {
        int turnDirection = (color == Alliance.BLUE) ? -1 : 1;

        double heading = 0;

        heading=0; //start at this heading
        gyroDrive(DRIVE_SPEED, 26, heading);

        // turn left
        heading=90 * turnDirection;
        gyroTurn(TURN_SPEED, heading);
        gyroHold(TURN_SPEED, heading, 0.5);

        // Drive to the line
        gyroDrive(DRIVE_SPEED, 20.0, heading);

        //
        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
