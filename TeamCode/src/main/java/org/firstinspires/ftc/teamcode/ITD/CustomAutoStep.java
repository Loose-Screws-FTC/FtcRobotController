package org.firstinspires.ftc.teamcode.ITD;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

class CustomAutoStep implements AutoStep {
    private Runnable doStart;
    private Runnable doRun;
    // One of the two fields will have a value
    private BooleanSupplier hasFinished = null;
    private double duration = 0;
    private ElapsedTime timer = new ElapsedTime();
    
    public CustomAutoStep(Runnable doStart, Runnable doRun, BooleanSupplier hasFinished) {
        this.doStart = doStart;
        this.doRun = doRun;
        this.hasFinished = hasFinished;
    }
    
    public CustomAutoStep(Runnable doStart, Runnable doRun, double duration) {
        this.doStart = doStart;
        this.doRun = doRun;
        this.duration = duration;
    }
    
    public void start() {
        if (duration != 0) {
            timer.reset();
        }
        doStart.run();
    }

    public void run(double deltaTime) {
        doRun.run();
    }
    
    public boolean hasFinished() {
        if (duration != 0) {
            return timer.time() >= duration;
        }
        if (hasFinished != null) {
            return hasFinished.getAsBoolean();
        }
        return false;
    }
}
