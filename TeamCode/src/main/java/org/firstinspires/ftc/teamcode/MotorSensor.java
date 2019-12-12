package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotSensor;

public abstract class MotorSensor extends RobotSensor {

    private DcMotor motor;

    public MotorSensor(DcMotor m) {
        this.motor = m;
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public boolean keepRunning() {
        super.keepRunning();
        return (motor.getCurrentPosition() > 5000);
    }
}
