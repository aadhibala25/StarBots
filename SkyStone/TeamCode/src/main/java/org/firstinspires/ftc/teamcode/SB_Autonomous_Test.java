package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="SB_Autonomous_Test", group="StarBots")
//@Disabled
public class SB_Autonomous_Test extends OpMode {

    final double INCHES_TO_TICS = 1440 / (4 * Math.PI);
    double targetDistanceInInches;
    double targetDistanceInTics;
    double currentDistanceInTics;
    double motorThreshold;
    Robot robot;

    public void init() {
        targetDistanceInInches = 12.0;
        targetDistanceInTics = INCHES_TO_TICS * targetDistanceInInches;
        currentDistanceInTics = 0;
        motorThreshold = 0.5;
        robot = new Robot(hardwareMap, telemetry);
    }

    public void loop() {
        currentDistanceInTics = robot.leftDrive.getCurrentPosition();
        if (currentDistanceInTics < targetDistanceInTics) {
            robot.setMovePower(motorThreshold, 0);
        }
        else {
            robot.setMovePower(0, 0);
        }
        telemetry.addData("Current Position (Tics)", "pos: %3f", currentDistanceInTics);
        telemetry.update();
    }
}