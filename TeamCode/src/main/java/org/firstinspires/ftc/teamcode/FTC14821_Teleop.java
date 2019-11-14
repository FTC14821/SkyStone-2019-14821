package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FTC 14821 Teleop", group = "")
public class FTC14821_Teleop extends LinearOpMode {

  private DcMotor leftDrive;
  private DcMotor rightDrive;
  private DcMotor armRotate;
  private CRServo rightGripper;
  private CRServo leftGripper;
  private DcMotor fangMotor;

  const double fastSpeed=0.6;
  const double slowSpeed=0.4;
  
  double gripClose;
  double gripOpen;
  double fangRelease;
  double fangGrab;
  double driveSpeedScale;

  /**
   * Describe this function...
   */
  private void initialize() {
    gripClose = 0;
    gripOpen = 1;
    fangRelease = 0.65;
    fangGrab = 0;
    leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightGripper.setDirection(DcMotorSimple.Direction.REVERSE);
    leftGripper.setPower(0);
    rightGripper.setPower(0);
    fangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    fangMotor.setPower(0);
    fangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    leftDrive = hardwareMap.dcMotor.get("leftDrive");
    rightDrive = hardwareMap.dcMotor.get("rightDrive");
    armRotate = hardwareMap.dcMotor.get("armRotate");
    rightGripper = hardwareMap.crservo.get("rightGripper");
    leftGripper = hardwareMap.crservo.get("leftGripper");
    fangMotor = hardwareMap.dcMotor.get("fangMotor");

    // Put initialization blocks here.
    initialize();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.left_bumper) {
          // Drive SLOW mode (normal)
          driveSpeedScale = slowSpeed;
        } else {
          // Drive FAST mode
          driveSpeedScale = fastSpeed;
        }
        leftDrive.setPower(-(driveSpeedScale * gamepad1.left_stick_y));
        rightDrive.setPower(-(driveSpeedScale * gamepad1.right_stick_y));
        armRotate.setPower(gamepad2.left_stick_y * 0.4);
        if (gamepad2.left_bumper == true) {
          leftGripper.setPower(1);
          rightGripper.setPower(1);
        } else if (gamepad2.right_bumper == true) {
          leftGripper.setPower(-1);
          rightGripper.setPower(-1);
        } else {
          leftGripper.setPower(0);
          rightGripper.setPower(0);
        }
        if (gamepad2.x == true) {
          fangMotor.setPower(0.1);
        } else if (gamepad2.y == true) {
          fangMotor.setPower(-0.1);
        } else {
          fangMotor.setPower(0);
        }
        telemetry.addData("fangPosition", fangMotor.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}
