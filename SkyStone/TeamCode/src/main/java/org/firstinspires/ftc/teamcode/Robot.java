package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public ElapsedTime runtime = new ElapsedTime();
    public final DcMotor leftDrive;
    public final DcMotor rightDrive;
    public final DcMotor armRotate;
    public final Servo claw_grab;
    public final Servo claw_move1;
    public final Servo claw_move2;
    public static final double MAX_POS  =  1.0;     // Maximum rotational position
    public static final double MIN_POS  =  -1.0;

    /**
     * Constructor that initializes the robot and all of its components and data members
     *
     * @param hardwareMap the HardwareMap object declared in the OpMode super class
     * @param telemetry the Telemetry object declared in the OpMode super class
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armRotate = hardwareMap.get(DcMotor.class, "arm_rotate");
        claw_grab = hardwareMap.get(Servo.class, "claw_grip");
        claw_move1 = hardwareMap.get(Servo.class, "claw_move1");
        claw_move2 = hardwareMap.get(Servo.class, "claw_move2");
        double  clawPosition = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        boolean rampUp = true;
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        armRotate.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
    }

    /**
     * Sets the power sent to the left and right motors based on the desired drive and turn powers
     *
     * @param drivePower the desired drive power: + for forward, - for reverse
     * @param turnPower the desired turn power: + for right, - for left
     */
    public void setMovePower(double drivePower, double turnPower) {
        // Calculate left and right wheel powers
        double leftPower = Range.clip(drivePower + turnPower, -1.0, 1.0);
        double rightPower = Range.clip(drivePower - turnPower, -1.0, 1.0);

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /**
     * Sets power to the arm motors based on the desired arm power
     *
     * @param power desired amount of power to be sent to the arm motor
     */
    public void setArmPower(double power) {
        power = Range.clip(power, -0.25, 0.25);
        armRotate.setPower(power);
        telemetry.addData("Motors", "armPower (%.2f)", power);
    }

    /**
     * Sets the position of the claw based on the desired claw position value
     *
     * @param hold desired hold position of the claw, between -0.5 and 0.5
     */
    public void setClawGrab(double hold) {
        hold = Range.clip(hold, -0.5, 0.5);
        claw_grab.setPosition(hold);
        telemetry.addData("Servo", "clawHold (%.2f)", hold);
    }

     /** Sets the position of the claw based on the desired claw position value
     *
             * @param hold desired hold position of the claw, between -0.5 and 0.5
            */
    public void setClawMove(double hold) {
        hold = Range.clip(hold, -0.5, 0.5);
        claw_move1.setPosition(hold);
        telemetry.addData("Servo", "clawMove1 (%.2f)", hold);
        claw_move2.setPosition(hold);
        telemetry.addData("Servo", "clawMove2 (%.2f)", hold);    }

}