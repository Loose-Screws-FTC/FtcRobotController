package org.firstinspires.ftc.teamcode.ITD;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

class Waypoint implements AutoStep {
    private double mX;
    private double mY;
    private double mYaw;
    private double mIntakePos;
    private double mIntakePower;
    private double mBucketPos;
    private boolean mJaw;
    private double mSlide;
    private Odometry mOdometry;
    private IMU mIMU;
    private Servo mIntakeServoLeft;
    private Servo mIntakeServoRight;
    private Servo mBucketServoLeft;
    private Servo mBucketServoRight;
    private Servo mJawServo;
    private DcMotor mLinearSlideL;
    private DcMotor mLinearSlideR;
    private DcMotor mLeftFrontDrive;
    private DcMotor mRightFrontDrive;
    private DcMotor mLeftBackDrive;
    private DcMotor mRightBackDrive;
    private double initialYaw;
    private double distanceFromTarget = Double.POSITIVE_INFINITY;

    private double toDegrees(double rad) {
        double deg = rad * 180.0 / Math.PI;
        return deg;
    }

    private double wrapToPI(double dYaw) {
        if (dYaw < -Math.PI)
            dYaw += Math.PI * 2; // fix wraparound
        if (dYaw > Math.PI)
            dYaw -= Math.PI * 2; // from +179 to -179

        return dYaw;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double getDist(double x0, double y0, double x1, double y1) {
        return Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    }

    private int getNull(Waypoint[] waypoints) {
        for (int i = 0; i < waypoints.length; i++) {
            if (waypoints[i] == null)
                return i;
        }
        return -1;
    }

    public Waypoint(double x, double y, double yaw, double intakePos, double intakePower, double bucketPos, boolean jaw, double slide, RobotHardware hardware) {
        mX = x;
        mY = y;
        mYaw = yaw;
        mIntakePos = intakePos;
        mIntakePower = intakePower;
        mBucketPos = bucketPos;
        mJaw = jaw;
        mSlide = slide;
        mOdometry = hardware.odometry;
        mIMU = hardware.imu;
        mIntakeServoLeft = hardware.intakeServoLeft;
        mIntakeServoRight = hardware.intakeServoRight;
        mBucketServoLeft = hardware.bucketServoLeft;
        mBucketServoRight = hardware.bucketServoRight;
        mJawServo = hardware.clawServo;
        mLinearSlideL = hardware.backElevatorLeft;
        mLinearSlideR = hardware.backElevatorRight;
        mLeftFrontDrive = hardware.leftFrontDrive;
        mRightFrontDrive = hardware.rightFrontDrive;
        mLeftBackDrive = hardware.leftBackDrive;
        mRightBackDrive = hardware.rightBackDrive;
        initialYaw = RobotHardware.initialYaw;
    }

    public double getX() {
        return mX;
    }
    public double getY() {
        return mY;
    }
    public double getYaw() {
        return mYaw;
    }
    public double getIntakePos() {
        return mIntakePos;
    }
    public double getIntakePower() {
        return mIntakePower;
    }
    public double getBucketPos() {
        return mBucketPos;
    }
    public boolean getJaw() {
        return mJaw;
    }
    public double getSlide() {
        return mSlide;
    }

    public void start() {
        elapsedTime = 0;
    }

    PIDController yawPID = new PIDController(0.2, 0, 0.05, -1, 1);
    public PIDController movePID = new PIDController(0.05, 0.0, 0.01, 0, 1);
    private double bailTime = 8;
    private double elapsedTime = 0;

    public void run(double timeDelta) {
        elapsedTime += timeDelta;

        double speed = 1;
        Odometry odometry = mOdometry;
        IMU imu = mIMU;
        Servo intakeServoLeft = mIntakeServoLeft;
        Servo intakeServoRight = mIntakeServoRight;
        Servo bucketServoLeft = mBucketServoLeft;
        Servo bucketServoRight = mBucketServoRight;
        Servo jawServo = mJawServo;
        DcMotor linearSlideL = mLinearSlideL;
        DcMotor linearSlideR = mLinearSlideR;
        DcMotor leftFrontDrive = mLeftFrontDrive;
        DcMotor rightFrontDrive = mRightFrontDrive;
        DcMotor leftBackDrive = mLeftBackDrive;
        DcMotor rightBackDrive = mRightBackDrive;

        double x = getX();
        double y = getY();
        double yaw = getYaw();
        double intakePos = getIntakePos();
        double intakePower = getIntakePower();
        double bucketPos = getBucketPos();
        boolean jawIsJawing = getJaw();
        double slidePos = getSlide();

        double currX = odometry.positionX();
        double currY = odometry.positionY();
        double currYawRad = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double currYaw = toDegrees(currYawRad);
        double yawRad = Math.toRadians(yaw);
        double currIntakePos = intakeServoLeft.getPosition();
        double currBucketPos = bucketServoLeft.getPosition();
        boolean currJaw = jawServo.getPosition() == 1;
        double currSlide = linearSlideR.getCurrentPosition();

        double yaw0 = getDist(Math.cos(currYawRad + .5), Math.sin(currYawRad + .5), Math.cos(yawRad), Math.sin(yawRad));
        double yaw1 = getDist(Math.cos(currYawRad - .5), Math.sin(currYawRad - .5), Math.cos(yawRad), Math.sin(yawRad));

        double newYaw;
        if (yaw0 > yaw1) {
            newYaw = -getDist(Math.cos(currYawRad), Math.sin(currYawRad), Math.cos(yawRad), Math.sin(yawRad)) / 1;
        } else {
            newYaw = getDist(Math.cos(currYawRad), Math.sin(currYawRad), Math.cos(yawRad), Math.sin(yawRad)) / 1;
        }

        double yawError = wrapToPI(yawRad - (currYawRad - initialYaw));
        double yawPower = Math.copySign(yawPID.calculate(0, yawError, timeDelta), newYaw);

        double direction = toDegrees(Math.atan2(currY - y, currX - x)) + 180 - (currYaw - initialYaw / Math.PI * 180);

        double moveError = getDist(currX, currY, x, y);
        double movePower = movePID.calculate(0, moveError, timeDelta);

        double powerX = Math.cos(Math.toRadians(direction)) * movePower;
        double powerY = -Math.sin(Math.toRadians(direction)) * movePower;

        linearSlideL.setPower(-(slidePos - currSlide) / 200);
        linearSlideR.setPower((slidePos - currSlide) / 200);

        /*if (intakePos > currIntakePos) {
            intakeServoLeft.setPosition(currIntakePos + intakePower);
            intakeServoRight.setPosition(currIntakePos + intakePower);
        } else if (intakePos < currIntakePos) {
            intakeServoLeft.setPosition(currIntakePos - intakePower);
            intakeServoRight.setPosition(currIntakePos - intakePower);
        }

        if (bucketPos > currBucketPos) {
            bucketServoLeft.setPosition(currBucketPos + .01);
            bucketServoRight.setPosition(currBucketPos + .01);
        } else if (bucketPos < currBucketPos) {
            bucketServoLeft.setPosition(currBucketPos - .01);
            bucketServoRight.setPosition(currBucketPos - .01);
        }*/

        if (jawIsJawing) {
            jawServo.setPosition(0.6);
        } else {
            jawServo.setPosition(0);
        }

        RobotHardware.telemetry.addData("powerX", powerX);
        RobotHardware.telemetry.addData("powerY", powerY);
        RobotHardware.telemetry.addData("yawPower", yawPower);

        double leftFrontPower =   powerX - powerY - yawPower;
        double rightFrontPower = -powerX - powerY + yawPower;
        double leftBackPower =   -powerX - powerY - yawPower;
        double rightBackPower =   powerX - powerY + yawPower;

        if (Math.abs(leftFrontPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(leftBackPower) > 1 || Math.abs(rightBackPower) > 1) {
            double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        distanceFromTarget = getDist(currX, currY, x, y)
            + Math.abs(currYaw - yaw) / 20
            + Math.abs(slidePos - currSlide) / 100;
    }

    public boolean hasFinished() {
        return (distanceFromTarget < 1 || elapsedTime >= bailTime);
    }
}