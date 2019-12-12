package org.firstinspires.ftc.teamcode;

import java.lang.Cloneable;

public abstract class RobotSensor implements Cloneable {

    public RobotSensor() {
    }

    public void init() {
    }

    public void update() {

    }

    /**
     * keepRunning()
     * return True if actions for this sensor should keep running
     * @return true by default, so a stubbed implementation is effectively transparent
     */
    public boolean keepRunning() {
        return true;
    }

}
