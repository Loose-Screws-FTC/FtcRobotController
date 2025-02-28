package org.firstinspires.ftc.teamcode.ITD;

public interface AutoStep {
    public void start();
    public void run(double deltaTime);
    public boolean hasFinished();
}
