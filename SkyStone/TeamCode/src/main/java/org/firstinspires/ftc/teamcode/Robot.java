package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public ElapsedTime runtime = new ElapsedTime();
    public final DcMotor leftDrive;
    public final DcMotor rightDrive;
    public final DcMotor armRotate;
    public final Servo claw;
    public static final double MAX_POS  =  1.0;     // Maximum rotational position
    public static final double MIN_POS  =  -1.0;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armRotate = hardwareMap.get(DcMotor.class, "arm_rotate");
        claw = hardwareMap.get(Servo.class, "claw_grip");
        double  clawPosition = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        boolean rampUp = true;
    }

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

    public void setClaw(double hold) {
        hold = Range.clip(hold, -0.5, 0.5);
        claw.setPosition(hold);
        telemetry.addData("Servo", "clawHold (%.2f)", hold);
    }
}