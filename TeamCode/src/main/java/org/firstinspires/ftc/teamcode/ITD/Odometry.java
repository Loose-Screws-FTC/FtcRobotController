package org.firstinspires.ftc.teamcode.ITD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Odometry {
    private IMU imu;
    private DcMotor odomX;
    private DcMotor odomY;
    private double prevYaw;
    private double prevOdomX;
    private double prevOdomY;

    // some constants
    double rX = 4.5;  // perpendicular distance of odom wheels from bot center
    double rY = 2.5;  // positive rotation (CCW) moves X and Y in - direction
    // double odomDiam = 1.92; // diameter of odometer wheels in inches
    double odomDiam = 1.92;
    double odomCountsPerRev = 8192;  // odometer counts per revolution

    // field position in inches, relative to initial position and orientation
    private double Xabs;
    private double Yabs;

    public double getFacing() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    public void init(IMU p_imu, DcMotor p_odomX, DcMotor p_odomY) {
        imu = p_imu;
        odomX = p_odomX;
        odomY = p_odomY;
        Xabs = 0;
        Yabs = 0;

        imu.resetYaw();
        odomX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odomY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odomX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odomY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // get initial yaw and odometer readings
        prevYaw = getFacing();
        prevOdomX = odomX.getCurrentPosition();
        prevOdomY = odomY.getCurrentPosition();
    }

    

    public void update() {

        // Get current Yaw (radians +CCW) and odometer readings (counts)
        double currYaw = getFacing();
        double currOdomX = -odomX.getCurrentPosition();     // mounted backwards
        double currOdomY = odomY.getCurrentPosition();

        // get changes      le
        double dYaw = currYaw - prevYaw;        // radians
        if (dYaw < -Math.PI) dYaw += 2 * Math.PI;   // fix wraparound
        if (dYaw > Math.PI) dYaw -= 2 * Math.PI;    // from +179 to -179
        double dOdomX = currOdomX - prevOdomX;  // counts
        double dOdomY = currOdomY - prevOdomY;  // counts

        // update prev values
        prevYaw = currYaw;
        prevOdomX = currOdomX;
        prevOdomY = currOdomY;

        // compute odometer motions (inches) due to wheel rotation
        double dOdomXinch = dOdomX * odomDiam * Math.PI / odomCountsPerRev;
        double dOdomYinch = dOdomY * odomDiam * Math.PI / odomCountsPerRev;

        // compute motions of odometers due to yaw rotation
        // adjust odometer motions by removing yaw components
        // RobotHardware.telemetry.addData("dYaw", dYaw);
        double dRotXinch = dYaw * rX;
        dOdomXinch -= dRotXinch;
        double dRotYinch = dYaw * rX;
        dOdomYinch -= dRotYinch;
        // RobotHardware.telemetry.addData("total X move", dOdomXinch);
        // RobotHardware.telemetry.addData("total Y move", dOdomYinch);

        // update field position
        double dXabs = dOdomXinch * Math.cos(currYaw) - dOdomYinch * Math.sin(currYaw);
        double dYabs = dOdomXinch * Math.sin(currYaw) + dOdomYinch * Math.cos(currYaw);
        Xabs += dXabs;
        Yabs += dYabs;
    }
    
    public Vector2 position() {
        return new Vector2(Xabs, Yabs);
    }

    public double positionX() {
        return Xabs;
    }

    public double positionY() {
        return Yabs;
    }
}
