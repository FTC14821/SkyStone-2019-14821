package org.firstinspires.ftc.teamcode;

import java.lang.Cloneable;

public abstract class RobotAction implements Cloneable {

    public RobotAction() {
    }

    public void initialize() {
    }

    public void update() {

    }

    public boolean keepRunning() {
        return false;
    }
}
