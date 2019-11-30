package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SB_BasicOpMode_Iterative_Test extends OpMode {

    Robot robot;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Reads input from conntrollers
        double drive = gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double armPower = gamepad2.left_stick_y * 0.25;
        double hold = gamepad2.right_trigger * 0.5;

        // Sets values of
        robot.setMovePower(drive, turn);
        robot.setArmPower(armPower);
        robot.setClaw(hold);
    }
}
