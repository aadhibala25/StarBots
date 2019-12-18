package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="SB_Autonomous_Test", group="StarBots")
@Disabled
public class SB_Autonomous_Test extends LinearOpMode {

    final double INCHES_TO_TICS = 1440 / (4 * Math.PI);

    double targetDistanceInInches = 32.0;
    double targetDistanceInTics = INCHES_TO_TICS * targetDistanceInInches;
    double currentDistanceInTics = 0;
    double motorThreshold = 0.5;
    Robot robot = new Robot(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        currentDistanceInTics = robot.leftDrive.getCurrentPosition();
        while (currentDistanceInTics < targetDistanceInTics) {
            robot.setMovePower(motorThreshold, 0);
        }
    }
}