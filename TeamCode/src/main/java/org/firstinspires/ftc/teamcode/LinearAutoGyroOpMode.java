package org.firstinspires.ftc.teamcode;

/**
 * LinearAutoGyroOpMode
 * This abstract class extends LinearGyroOpMode and implements the methods for our autonomous programs.
 * Each autonomous opmode will be a subclass of this one and call a method which has the actual movement
 *
 * <p>The methods allow you to specify your Alliance color to decide directions
 *
 * @author team14821 Cactus Intelligence Agency
 */
public abstract class LinearAutoGyroOpMode extends LinearGyroOpMode {

    public enum Direction {
        LEFT, RIGHT
    }

    public double AUTO_DRIVE_FAST = 0.8;     // Nominal speed for better accuracy.
    public double AUTO_DRIVE_SLOW = 0.6;     // Nominal speed for better accuracy.
    public double AUTO_TURN_SPEED = 0.6;     // Nominal half speed for better accuracy.

    public static int WALL_TO_BLOCK_DISTANCE = 32;
    public static int BLOCK_STOP_DISTANCE = 7;
    public static int BLOCKSIZE = 8;

    /**
     * Scoot to the left or right the distance of 1 block but at any angle not just 90 degrees
     * Back up so we don't bump into the block in front of us, with extra distance depending on our angle
     * Turn (left or right) by angle degrees
     * Drive forward the hypotenuse of our triangle dH
     * Turn back to the original heading
     * Drive forward, stopping if we get too close to a block (distance sensor)
     *
     * @param direction   Which direction we should scoot
     *
     */
    public void scootAngle(Direction direction, double angle) {
        double turnAngle = (Math.abs(angle) <= 90 && angle != 0) ? Math.abs(angle) : 90; //default to 90
        double minBack = 5; // minimum distance backwards to drive

        // calculate the distance backwards (dY) and alongthe hypotenuse (dH) for the angle we want to turn
        // and avoid dividing by zero or infinity
        double dY = (turnAngle != 90) ? Math.abs(BLOCKSIZE / Math.tan(Math.toRadians(turnAngle))) : 0; //how far back to we have to go to get our mark
        double dH = Math.abs(BLOCKSIZE / Math.sin(Math.toRadians(turnAngle))); //already corrected for turnAngle > 0 so it's safe

        if (direction == Direction.RIGHT) {
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
        gyroDrive(AUTO_DRIVE_SLOW, dH, heading, 3);

        // turn back to original heading
        heading = originalHeading;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);

        // now drive forward the minBack distance
        gyroDrive(AUTO_DRIVE_SLOW, minBack, heading, 3, 6);
    }

    /**
     * Scoot to the left or right the distance of 1 block
     * Back up so we don't bump into the block in front of us
     * Turn (left or right) 90 degrees
     * Drive forward the distance of 1 block
     * Turn back to the original heading
     * Drive forward, stopping if we get too close to a block (distance sensor)
     *
     * @param direction   Which direction we are scooting, LEFT or RIGHT
     *
     */
    public void scoot(Direction direction) {

        // can just replace with this really
//        scootAngle(direction, 90);

        double turnAngle = 90;
        if (direction == Direction.RIGHT) {
            turnAngle *= -1;
        }
        double originalHeading = lastHeading;
        double heading = originalHeading;

        gyroDrive(AUTO_DRIVE_SLOW, -5, heading);
        heading += turnAngle;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroDrive(AUTO_DRIVE_SLOW, BLOCKSIZE, heading, 1);
        heading = originalHeading;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, 5, heading, 2, 6);
//        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
    }

    /**
     * Autonomous to get both skystones.  Start with robot aimed at the first block that's farthest
     * from the wall (position 0).
     * Drive close to the block fast, then drive slower until we're close to it.
     * Check the color - if it's not a skystone then scoot to the left or right and check again
     * If we get to the third block and don't recognize a skystone, just pick it up and continue
     * Back up, turn (left or right) and drive fast to the other side, then drop the block
     * Back all the way to the position of th next skystone
     * If the first skystone was in position 2, then just go back to position 4 since we can't pick up
     * a block that's against the wall anyhow
     * After delivering the second block, just back up to the line
     *
     * @param color   Which alliance color we are, determines the direction for scooting and
     *                turning to put the block on the other side
     *
     */

    public void autoTwoSkystones(Alliance color) {
        double heading = 0;
        double blockPosition;
        double longDistance = 38;

        Direction scootDirection = (color == Alliance.BLUE) ? Direction.RIGHT : Direction.LEFT;
        int turnDirection = (color == Alliance.BLUE) ? 1 : -1;

        heading = 0; //start at this heading
        robot.openGripper();
        gyroDrive(AUTO_DRIVE_SLOW, 28, heading, 10, 10);
        gyroDrive(AUTO_DRIVE_SLOW*0.75, 4, heading, 10, 10);
        blockPosition = 0;
        while (!isSkystone() && blockPosition < 2) {
            scoot(scootDirection);
            blockPosition++;
        }
        robot.closeGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, 1);
        robot.moveArmToPosition(1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, -8.0, heading, 1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // turn right and drive to the other side
        heading = 90 * turnDirection;    //LEFT turn postive, RIGHT turn negative
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_FAST, longDistance + blockPosition * BLOCKSIZE, heading, 10);
        // spit the block out by running grippers for 0.75 seconds
        robot.openGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // now go  back and get next skystone
        blockPosition = (blockPosition < 2) ? blockPosition+3 : 4;

        gyroDrive(AUTO_DRIVE_FAST, -(longDistance + blockPosition * BLOCKSIZE), heading, 10);
        robot.moveArmToPosition(0); //lower arm
        heading = 0;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);

        //drive up to the next stone and start looking
        gyroDrive(AUTO_DRIVE_SLOW*0.75, 8.0, heading, 2, 10);
        robot.closeGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, 1); //wait for gripper to close
        robot.moveArmToPosition(1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, -8.0, heading, 2);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // turn right and drive to the other side
        heading = 90 * turnDirection;    //LEFT turn postive, RIGHT turn negative
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_FAST, longDistance + blockPosition * BLOCKSIZE, heading, 10);
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

    /**
     * Go get the first block and take it across the line, the park on line and stop
     * This isn't used anymore since we can get two skystones
     *
     * @param color our Alliance color
     */
    public void autoGetFirstBlock(Alliance color) {

        int turnDirection = (color == Alliance.BLUE) ? 1 : -1;

        double heading = 0;

        heading = 0; //start at this heading
        robot.openGripper();
        gyroDrive(AUTO_DRIVE_SLOW, 32.0, heading, 10, 6);
        robot.closeGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, 1);
        robot.moveArmToPosition(1);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW, -2.0, heading, .5);
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        // turn right and drive to the other side
        heading = 90 * turnDirection;    //LEFT turn postive, RIGHT turn negative
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, HOLD);
        gyroDrive(AUTO_DRIVE_SLOW * 2, 40.0, heading, 3);

        // spit the block out by running grippers for 0.75 seconds
        robot.moveArmToPosition(0);
        robot.openGripper();
        gyroHold(AUTO_DRIVE_SLOW, heading, HOLD);

        gyroDrive(AUTO_DRIVE_SLOW, -16.0, heading);
        gyroHold(AUTO_DRIVE_SLOW, heading, 1);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /**
     * Drive forward to the bridge, turn, and park on line
     * Starting on the FOUNDATION side, so BLUE turns RIGHT (-1)
     * RED turns LEFT (+1)
     *
     * @param color our Alliance color
     */

    public void autoParkOnLineAtBridge(Alliance color) {
        int turnDirection = (color == Alliance.BLUE) ? -1 : 1;

        double heading = 0;

        heading = 0; //start at this heading
        gyroDrive(AUTO_DRIVE_SLOW, 26, heading);

        // turn left
        heading = 90 * turnDirection;
        gyroTurn(AUTO_TURN_SPEED, heading);
        gyroHold(AUTO_TURN_SPEED, heading, 0.5);

        // Drive to the line
        gyroDrive(AUTO_DRIVE_SLOW, 20.0, heading);

        //
        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    /**
     * Move the foundation into the build area (the corner)
     * Start with the fangs against the wall
     * Drive forward, turn around, then back into the foundation
     * Grab the foundation with our fangs, then drive forward
     * Turn toward the line which will turn the foundation into the corner
     * Let go and then drive torward the line
     *
     * @param color Which color our Alliance is
     */
    public void autoMoveFoundation(Alliance color) {
        int turnDirection = (color == Alliance.BLUE) ? 1 : -1;

        double heading = 0;

        gyroDrive(AUTO_DRIVE_FAST, 20, heading);
        heading = 180;
        gyroTurn(AUTO_TURN_SPEED, heading);
        robot.openFangs();
        gyroDrive(AUTO_DRIVE_SLOW, -24, heading);
        robot.closeFangs();
        sleep(1000);
        gyroDrive((AUTO_DRIVE_SLOW+AUTO_DRIVE_FAST)/2, 30, heading);
        heading += 30 * turnDirection;
        gyroTurn(AUTO_TURN_SPEED, heading, 5);
        robot.openFangs();
        heading += 60 * turnDirection;
        gyroTurn(AUTO_TURN_SPEED, heading, 3);
        gyroDrive(AUTO_DRIVE_FAST, 30, heading);
        robot.closeFangs();
        sleep(1000);
    }

}
