package org.firstinspires.ftc.teamcode.ITD;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

class DelayAutoStep implements AutoStep {
    private double delay;
    private RobotHardware hardware;
    private ElapsedTime timer = new ElapsedTime();
    
    public DelayAutoStep(double delay, RobotHardware hardware) {
        this.delay = delay;
        this.hardware = hardware;
    }
    
    public void start() {
        timer.reset();
    }

    public void run(double deltaTime) {
        hardware.leftFrontDrive.setPower(0);
        hardware.rightFrontDrive.setPower(0);
        hardware.leftBackDrive.setPower(0);
        hardware.rightBackDrive.setPower(0);
    }
    
    public boolean hasFinished() {
        return timer.time() >= delay;
    }
}
