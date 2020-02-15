package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/*
 * RobotSettings contains the defaults used throughout the opModes that can be overridden by
 * values in a file
 * Create a new instance of the RobotSettings class
 *      RobotSettings settings = new RobotSettings();
 * then load the settings from the file
 *      settings.readSettings();
 */

public class RobotSettings {

//    public static String settingsFileName = "CactusRobotSettings.json";

    private final static String SETTINGS_FILE = "CactusRobotSettings.json";
    public int version = 1;  //version of this class
    public double teleFastSpeed = 0.7;  //fast drive speed for teleOp
    public double teleSlowSpeed = 0.4;  //slow drive speed for teleop

    public double autoDriveSpeed = 0.4; //drive speed for autonomous
    public double autoTurnSpeed = 0.3;  // turn speed for autonomous
    public double autoHeadingThreshold = 1; //as tight as we can make it with an integer gyro
    public double autoTurnCoefficient = 0.1; //large is more responsive, but also less stable
    public double autoDriveCoefficient = 0.15; //large is more responsive, but also less stable

    public double driveEncoderCountsPerRev = 753.2;   //goBilda "fast" motors
    public double driveGearRatio = 1.0; //Make < 1.0 f geared up
    public double driveWheelDiameter = 4.0;     //diameter of wheel in inches


    public RobotSettings() {
    }

    public void readSettings() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(this.SETTINGS_FILE);
            String data = ReadWriteFile.readFile(file);
            Log.d("CACTUS", "Loaded Settings: ");
            RobotSettings x = SimpleGson.getInstance().fromJson(data, RobotSettings.class);
            this.teleFastSpeed = x.teleFastSpeed;
            this.teleSlowSpeed = x.teleSlowSpeed;
            this.autoDriveSpeed = x.autoDriveSpeed;
            this.autoTurnSpeed = x.autoTurnSpeed;
            this.autoHeadingThreshold = x.autoHeadingThreshold;
            this.autoTurnCoefficient = x.autoTurnCoefficient;
            this.autoDriveCoefficient = x.autoDriveCoefficient;
            this.driveEncoderCountsPerRev = x.driveEncoderCountsPerRev;
            this.driveGearRatio = x.driveGearRatio;
            this.driveWheelDiameter = x.driveWheelDiameter;
        }
        catch (Exception e)
        {
            Log.w("CACTUS", "error reading settings file %s: ", e);
        }
    }

    public void writeSettings() {
        File file = AppUtil.getInstance().getSettingsFile(this.SETTINGS_FILE);
        ReadWriteFile.writeFile(file, this.serialize());
    }

    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }

    public static RobotSettings deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, RobotSettings.class);
    }

}
