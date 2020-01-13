package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="StarBots TeleOpMode", group="StarBots")
//@Disabled
public class SB_BasicOpMode_Iterative_Test extends OpMode {

    private Robot robot;
    boolean clawGrabbed = true;
    boolean clawPush = true;

    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.setClawGrab(0.0);
        robot.setClawMove(0.0);
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
//        double drive = gamepad1.left_trigger - gamepad1.right_trigger;
        double drive = gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double armPower = gamepad2.left_stick_y * 0.3;

        // Reverse & slow mode
        if (gamepad1.right_trigger >= 0.25) {
            drive *= 0.35;
            turn *= 0.4;
        }
        if (gamepad1.left_trigger >= 0.25) {
            drive *= -1;
        }

        // Opens/closes claws
        if (!clawGrabbed) {
            if (gamepad2.right_bumper)
                clawGrabbed = true;
        }
        if (clawGrabbed) {
            if (gamepad2.left_bumper)
                clawGrabbed = false;
        }


        // Opens/closes claws

        if (!clawPush) {
            if (gamepad2.b)
                clawPush = true;
        }
        if (clawPush) {
            if (gamepad2.a)
                clawPush = false;
        }



        // Sets values of motors/servos
        robot.setMovePower(drive, turn);
        robot.setArmPower(armPower);
        robot.setClawGrab(clawGrabbed ? 0 : 0.5);
        robot.setClawMove(clawPush ? 0 : 0.5);
    }
}
