package org.firstinspires.ftc.teamcode.ITD;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.TouchSensor;

// Controller scheme: https://www.padcrafter.com/?templates=LM3+Scheme&plat=0&yButton=Override+slide+limit&leftTrigger=Vertical+slide+down&rightTrigger=Vertical+slide+up&leftBumper=Intake+spin+out&rightBumper=Intake+spin+in&col=%23242424%2C%23606A6E%2C%23FFFFFF&xButton=Light+pattern+1&bButton=Light+pattern+2&leftStickClick=&leftStick=Move&rightStickClick=Turn&rightStick=Change+team+color

// Want this scheme: https://www.padcrafter.com/?templates=Controller+Scheme+1&plat=0&col=%23242424%2C%23606A6E%2C%23FFFFFF&rightTrigger=Towers+Up&rightBumper=Specimen+Claw+Grab&leftBumper=Specimen+Claw+Release&leftTrigger=Towers+Down&leftStickClick=&bButton=Intake+Eject&xButton=Intake+Align+Vertical+Sample&yButton=Lower+Intake&aButton=Raise+Intake&rightStick=Rotational+Movement+%2F+Extend+%26+Retract+Intake&leftStick=Lateral+Movement+%28Field+Relative%29&dpadUp=Lower+Bucket&dpadDown=Raise+Bucket&dpadRight=Move+to+Human+Player&dpadLeft=Move+to+Submersible


@TeleOp(name = "TeleOp Qualifier", group = "Linear OpMode")
public class TeleOpQualifier extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robotHardware;
    private boolean overrideSlideLimit = false;
    private boolean isIntakeDown = false;
    private float bottomEncoderPositionL = 0;
    private float bottomEncoderPositionR = 0;
    private float timeUntilIntakeMovesBack = 0;
    private double reversePower;

    // DeltaTime calculation
    private double deltaTime;
    private double lastTime = 0;

    private double timeSinceIntakeDown = 1;
    private double timeSinceOuttakeOut = 1;

    private boolean intakeModBoolean = false;

    private double verticalSlideDirection = 0;

    private double intakeSlidePower = 0;
    private boolean clawClosed = true;
    private boolean bucketActive = false;

    private int intakePositionState = 0;
    private double intakePower = 0;

    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    private boolean elevatorHoldingMaximum = false;

    private class AutomationData {
        public String[] commands;
        public int currentCommandIndex = 0;
        public double timeSinceLastCommand = 0;

        AutomationData(String[] commands) {
            this.commands = commands;
        }
    }

    private AutomationData currentAutomationData = null;

    private void InitializeHardware() {
        robotHardware = new RobotHardware(hardwareMap);
        bottomEncoderPositionL = robotHardware.backElevatorLeft.getCurrentPosition();
        bottomEncoderPositionR = robotHardware.backElevatorRight.getCurrentPosition();
    }

    private void InitializeServos() {
        ((PwmControl) robotHardware.intakeServoLeft).setPwmEnable();
        ((PwmControl) robotHardware.intakeServoRight).setPwmEnable();
        ((PwmControl) robotHardware.bucketServoLeft).setPwmEnable();
        ((PwmControl) robotHardware.bucketServoRight).setPwmEnable();
        ((PwmControl) robotHardware.outtakeGuardServo).setPwmEnable();
        ((PwmControl) robotHardware.clawServo).setPwmEnable();
        robotHardware.intakeServoLeft.setPosition(RobotHardware.lServoIn);
        robotHardware.intakeServoRight.setPosition(RobotHardware.rServoIn);
        robotHardware.bucketServoLeft.setPosition(RobotHardware.lServoOutIn);
        robotHardware.bucketServoRight.setPosition(RobotHardware.rServoOutIn);
        robotHardware.outtakeGuardServo.setPosition(RobotHardware.guardServoDown);
    }

    private void CalculateDeltaTime() {
        deltaTime = runtime.seconds() - lastTime;
        lastTime = runtime.seconds();
    }

    // Automation formatting
    // Command:
    // "duration:drivex:drivey:driveyaw:intakeslide:verticalslide:intakeposition:intakepower:bucketposition:clawposition"

    private void ProcessAutomation(double deltaTime) {
        if (currentAutomationData == null) return;
        if (currentAutomationData.commands.length == 0) return;
        if (currentAutomationData.currentCommandIndex >= currentAutomationData.commands.length || gamepad1.b || gamepad1.right_trigger > 0.05) {
            currentAutomationData = null;
            return;
        }
        String command = currentAutomationData.commands[currentAutomationData.currentCommandIndex];
        
        // Parse the command
        String[] parts = command.split(":");
        if (parts.length != 10) {
            currentAutomationData = null;
            return;
        }
        double duration = Double.parseDouble(parts[0]);
        double drivex = Double.parseDouble(parts[1]);
        double drivey = Double.parseDouble(parts[2]);
        double driveyaw = Double.parseDouble(parts[3]);
        double intakeslide = Double.parseDouble(parts[4]);
        double verticalslide = Double.parseDouble(parts[5]);
        double intakeposition = Double.parseDouble(parts[6]);
        double intakepower = Double.parseDouble(parts[7]);
        double bucketposition = Double.parseDouble(parts[8]);
        double clawposition = Double.parseDouble(parts[9]);

        // Drive
        double axial = drivey;
        double lateral = drivex;
        double yaw = driveyaw;

        if (axial != 0 || lateral != 0 || yaw != 0) {
            // Compensate for robot orientation
            YawPitchRollAngles robotOrientation = robotHardware.imu.getRobotYawPitchRollAngles();
            double robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS) - robotHardware.initialYaw + Math.PI;
            double radians = Math.atan2(axial, lateral);
            double power = Math.sqrt(axial * axial + lateral * lateral);

            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
        }

        // Intake
        intakePositionState = (int) intakeposition;
        if (intakePositionState == 2) isIntakeDown = true;
        else isIntakeDown = false;

        // Intake slide
        intakeSlidePower = intakeslide;

        // Vertical slide
        verticalSlideDirection = verticalslide;

        // Claw
        clawClosed = clawposition > 0.5;

        // Bucket
        bucketActive = bucketposition > 0.5;

        // Intake power
        intakePower = intakepower;

        // See if we need to move to the next command
        currentAutomationData.timeSinceLastCommand += deltaTime;
        if (currentAutomationData.timeSinceLastCommand >= duration) {
            currentAutomationData.currentCommandIndex++;
            currentAutomationData.timeSinceLastCommand = 0;
            return;
        }
    }

    @Override
    public void runOpMode() {
        InitializeHardware();
        RobotHardware.telemetry = telemetry;
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        InitializeServos();

        // ============================================================================
        // Update Loop (until driver presses stop)
        // ============================================================================
        while (opModeIsActive()) {
            CalculateDeltaTime();
            timeSinceIntakeDown += deltaTime;
            timeSinceOuttakeOut += deltaTime;

            double max;
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftBackPower = 0;
            rightBackPower = 0;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            if (!RobotHardware.povDrive) {
                YawPitchRollAngles robotOrientation = robotHardware.imu.getRobotYawPitchRollAngles();
                double robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS) - robotHardware.initialYaw + Math.PI;
                double radians = Math.atan2(axial, lateral);
                double power = Math.sqrt(axial * axial + lateral * lateral);
                if (power > 0.05) {
                    power = Math.max(Math.pow(power, 2), 0.2);
                } else {
                    power = 0;
                }
                telemetry.addData("Power", power);
                radians -= robotYaw;
                if (radians > 2 * Math.PI) radians -= 2 * Math.PI;
                if (radians < 0) radians += 2 * Math.PI;
                axial = Math.sin(radians) * power;
                lateral = Math.cos(radians) * power;
            }

            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            intakeSlidePower = gamepad1.right_stick_y;
            if (gamepad1.y) {
                isIntakeDown = true;
            } else if (gamepad1.a || gamepad1.x) {
                isIntakeDown = false;
            }
            if (gamepad1.y) {
                intakePositionState = 2;
            } else if (gamepad1.a) {
                intakePositionState = 0;
            } else if (gamepad1.x) {
                intakePositionState = 1;
            }
            if (gamepad1.right_trigger > 0) {
                verticalSlideDirection = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0) {
                verticalSlideDirection = -gamepad1.left_trigger;
            } else {
                verticalSlideDirection = 0;
            }
            if (gamepad1.right_bumper) {
                clawClosed = true;
            }
            if (gamepad1.left_bumper) {
                clawClosed = false;
            }
            if (gamepad1.dpad_up) {
                bucketActive = true;
            } else {
                bucketActive = false;
            }
            intakePower = 0;
            if (gamepad1.b && (robotHardware.elevatorLimitLeft.isPressed() || robotHardware.elevatorLimitRight.isPressed())) {
                intakePower = -1;
            } else {
                reversePower = RobotHardware.reverseStartPower;
                if (isIntakeDown) {
                    timeSinceIntakeDown = 0;
                    intakePower = RobotHardware.intakeForwardPower;
                }
                if (timeSinceIntakeDown < RobotHardware.shutoffIntakeDelay) intakePower = RobotHardware.intakeForwardPower;
                if (gamepad1.b && (isIntakeDown || timeSinceIntakeDown < RobotHardware.shutoffIntakeDelay)) {
                    intakePower = -1;
                }
            }
            if (gamepad1.guide) {
                currentAutomationData = new AutomationData(
                        new String[]{
                                "0.25:0:0:0:0:-1:1:0:0:0",
                                "0.5:0:0:0:1:-1:1:0:0:0",
                                "0.75:0:0:0:0:-1:0:0:0:0",
                                "0.5:0:0:0:0:0:0:-1:0:0",
                        }
                );
            }

            // Process automation
            ProcessAutomation(deltaTime);

            // Check slide limits
            float currentEncoderL = robotHardware.backElevatorLeft.getCurrentPosition();
            float currentEncoderR = robotHardware.backElevatorRight.getCurrentPosition();
            if (robotHardware.elevatorLimitLeft.isPressed()) {
                bottomEncoderPositionL = currentEncoderL;
            }
            if (robotHardware.elevatorLimitRight.isPressed()) {
                bottomEncoderPositionR = currentEncoderR;
            }

            if (intakePositionState == 2) {
                robotHardware.intakeServoLeft.setPosition(RobotHardware.lServoOut);
                robotHardware.intakeServoRight.setPosition(RobotHardware.rServoOut);
            } else if (intakePositionState == 0) {
                robotHardware.intakeServoLeft.setPosition(RobotHardware.lServoIn);
                robotHardware.intakeServoRight.setPosition(RobotHardware.rServoIn);
                if (intakeModBoolean) {
                    robotHardware.intakeServoLeft.setPosition(RobotHardware.lServoInMod);
                    robotHardware.intakeServoRight.setPosition(RobotHardware.rServoInMod);
                }
            } else if (intakePositionState == 1) {
                robotHardware.intakeServoLeft.setPosition(RobotHardware.lServoMed);
                robotHardware.intakeServoRight.setPosition(RobotHardware.rServoMed);
            }

            if (isIntakeDown) {
                timeSinceIntakeDown = 0;
            }
            if (intakePower <= 0 && timeSinceIntakeDown < RobotHardware.shutoffIntakeDelay && !gamepad1.b) {
                intakePower = RobotHardware.intakeForwardPower;
            }
            robotHardware.intakeMotor.setPower(intakePower);

            double linearSlidePowerMultiplier = 1;

            if (currentEncoderL - bottomEncoderPositionL >= RobotHardware.maxSlidePosition || currentEncoderR - bottomEncoderPositionR <= -RobotHardware.maxSlidePosition) {
                if (!elevatorHoldingMaximum) {
                    robotHardware.backElevatorLeft.setTargetPosition((int) RobotHardware.maxSlidePosition + (int) bottomEncoderPositionL);
                    robotHardware.backElevatorRight.setTargetPosition((int) -RobotHardware.maxSlidePosition + (int) bottomEncoderPositionR);
                    robotHardware.backElevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robotHardware.backElevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                elevatorHoldingMaximum = true;
            }
            
            if (verticalSlideDirection > 0 && !elevatorHoldingMaximum) {
                // Up
                robotHardware.backElevatorLeft.setPower(verticalSlideDirection * linearSlidePowerMultiplier * (1 - (currentEncoderL - bottomEncoderPositionL >= RobotHardware.maxSlidePosition ? 1 : 0)));
                robotHardware.backElevatorRight.setPower(-verticalSlideDirection * linearSlidePowerMultiplier * (1 - (currentEncoderR - bottomEncoderPositionR <= -RobotHardware.maxSlidePosition ? 1 : 0)));
                intakeModBoolean = true;
            } else if (verticalSlideDirection < 0) {
                if (elevatorHoldingMaximum) {
                    robotHardware.backElevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robotHardware.backElevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    elevatorHoldingMaximum = false;
                }
                // Down
                robotHardware.backElevatorLeft.setPower(verticalSlideDirection * linearSlidePowerMultiplier * (1 - (robotHardware.elevatorLimitLeft.isPressed() ? 1 : 0)));
                robotHardware.backElevatorRight.setPower(-verticalSlideDirection * linearSlidePowerMultiplier * (1 - (robotHardware.elevatorLimitRight.isPressed() ? 1 : 0)));
            } else if (!elevatorHoldingMaximum) {
                // Stop
                robotHardware.backElevatorLeft.setPower(0);
                robotHardware.backElevatorRight.setPower(0);
            }

            if (robotHardware.elevatorLimitLeft.isPressed() || robotHardware.elevatorLimitRight.isPressed()) {
                intakeModBoolean = false;
                robotHardware.outtakeGuardServo.setPosition(RobotHardware.guardServoDown);
            } else {
                robotHardware.outtakeGuardServo.setPosition(RobotHardware.guardServoUp);
            }

            if (bucketActive) {
                timeSinceOuttakeOut = 0;
            }

            if (timeSinceOuttakeOut < robotHardware.guardDownAfterOuttakeDownDelay) {
                robotHardware.outtakeGuardServo.setPosition(RobotHardware.guardServoUp);
            }

            // State is inverted on this switch
            if ((intakeSlidePower > 0 && robotHardware.horizontalLimit.getState()) || intakeSlidePower <= 0) {
                robotHardware.intakeExtensionSlide.setPower(intakeSlidePower);
            } else {
                robotHardware.intakeExtensionSlide.setPower(0);
            }

            if (clawClosed) {
                robotHardware.clawServo.setPosition(RobotHardware.releasePosition);
            } else {
                robotHardware.clawServo.setPosition(RobotHardware.grabPosition);
            }

            if (bucketActive) {
                robotHardware.bucketServoLeft.setPosition(RobotHardware.lServoOutOut);
                robotHardware.bucketServoRight.setPosition(RobotHardware.rServoOutOut);
            } else {
                if (intakeModBoolean) {
                    robotHardware.bucketServoLeft.setPosition(RobotHardware.lServoOutInMod);
                    robotHardware.bucketServoRight.setPosition(RobotHardware.rServoOutInMod);
                } else {
                    robotHardware.bucketServoLeft.setPosition(RobotHardware.lServoOutIn);
                    robotHardware.bucketServoRight.setPosition(RobotHardware.rServoOutIn);
                }
            }

            // Send calculated power to wheels
            robotHardware.leftFrontDrive.setPower(leftFrontPower);
            robotHardware.rightFrontDrive.setPower(rightFrontPower);
            robotHardware.leftBackDrive.setPower(leftBackPower);
            robotHardware.rightBackDrive.setPower(rightBackPower);
            
            // Update the odometry
            robotHardware.odometry.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Position", "X: " + robotHardware.odometry.positionX() + " | Y: " + robotHardware.odometry.positionY());
            telemetry.addData("IMU Rotation", robotHardware.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("DeltaTime", deltaTime);
            telemetry.addData("ReversePower", reversePower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Slide Position", robotHardware.backElevatorRight.getCurrentPosition());
            telemetry.addData("Left Slide Limit", robotHardware.elevatorLimitLeft.isPressed());
            telemetry.addData("Right Slide Limit", robotHardware.elevatorLimitRight.isPressed());
            telemetry.addData("Horizontal Limit", robotHardware.horizontalLimit.getState());
            telemetry.addData("Holding Maximum", elevatorHoldingMaximum);
            telemetry.update();
        }
    }
}
