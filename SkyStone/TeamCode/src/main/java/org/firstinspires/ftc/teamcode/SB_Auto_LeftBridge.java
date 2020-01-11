/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Park LeftBridge", group="StarBots")
//@Disabled
public class SB_Auto_LeftBridge extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  0.025;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -0.025;     // Maximum REV power applied to motor

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armRotate = null;
    private Servo claw = null;
    private static final double MAX_POS  =  1.0;     // Maximum rotational position
    private static final double MIN_POS  =  -1.0;     // Minimum rotational position
    private static double distance = 0;
    double clawHold;

    // Define class members
    double  power   = 0;
    boolean rampUp  = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armRotate = hardwareMap.get(DcMotor.class, "arm_rotate");
        claw = hardwareMap.get(Servo.class, "claw_grip");
        clawHold     = Range.clip(-0.25, 0.5, -0.5) ;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armRotate.setDirection(DcMotor.Direction.FORWARD);
        claw.setPosition(clawHold);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //runtime.reset();
        //distance++;

        //Sleep for a second
        Thread.sleep(1000);

        //Go Forward
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        Thread.sleep(300);

        //Turn Left
        leftDrive.setPower(0.9);
        rightDrive.setPower(-0.9);
        Thread.sleep(600);

        //Go Forward
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        Thread.sleep(900);


        // Display the current value
        telemetry.addData("Motor Power", "%5.2f", power);
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

        // Set the motor to the new power and pause;
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        //armRotate.setPower(power);
        sleep(CYCLE_MS);
        idle();
        }
}
