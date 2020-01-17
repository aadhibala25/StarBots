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
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="Around_Foundation_Red", group="StarBots")
//@Disabled
public class SB_Auto_LeftBridge_Foundation_Around extends LinearOpMode {

    // Define OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  0.025;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -0.025;     // Maximum REV power applied to motor
    SB_Push_Robot         robot   = new SB_Push_Robot();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;
    //private DcMotor armRotate = null;
    //private Servo claw0 = null;
    //private Servo claw1 = null;
    //private Servo claw2 = null;

    private static final double MAX_POS  =  1.0;     // Maximum rotational position
    private static final double MIN_POS  =  -1.0;     // Minimum rotational position
    private static double distance = 0;
    double clawHold;

    // Define class members
    double  power   = 0;
    boolean rampUp  = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Auto Start Time", "%5.2f", getRuntime() );
        telemetry.update();

        robot.setClawMove(true ? 0 : 1.0);              //Open Claw
        Thread.sleep(1000);

        robot.move(0.2, 0.8, 300);      //Go Forward Rightish
        Thread.sleep(500);

        robot.move(0.9, 0.2, 300);      //Go Forward Leftish
        Thread.sleep(650);
        robot.move(0.0, 0.0, 100);      //Stop Robot
        robot.setClawMove(false ? 0 : 1.0);             //Close Claw
        Thread.sleep(1000);

        telemetry.addData("Pull Start Time", "%5.2f", getRuntime() );
        telemetry.update();

        robot.move(-0.25, -0.25, 1000);      //Pull Foundation back
        Thread.sleep(7000);
        //robot.setClawMove(false ? 0 : 1.0);             //Close Claw
        //Thread.sleep(3000);
        robot.move(0.25, 0.25, 500);      //Push Foundation forward little bit
        Thread.sleep(800);
        robot.setClawMove(true ? 0 : 1.0);             //Open Claw
        Thread.sleep(500);
        robot.move(-0.25, -0.25, 500);      //Go Backward little
        Thread.sleep(500);

        telemetry.addData("Pull End Time", "%5.2f", getRuntime() );
        telemetry.update();

        robot.move(1.0, -1.0, 700);     //Turn Left
        Thread.sleep(500);

        robot.move(0.5, 0.5, 700);      //Go Forward
        Thread.sleep(800);

        robot.setClawMove(false ? 0 : 1.0);             //Close Claw
        Thread.sleep(200);

        robot.move(-1.0, 1.0, 800);     //Turn Right
        Thread.sleep(550);

        robot.move(0.5, 0.5, 300);      //Go Forward
        Thread.sleep(2800);

        robot.move(-1.0, 1.0, 800);     //Turn Right
        Thread.sleep(500);

        robot.move(0.5, 0.5, 300);      //Go Forward
        Thread.sleep(1100);

        robot.move(-1.0, 1.0, 800);     //Turn Right
        Thread.sleep(500);

        robot.move(1.0, 1.0, 300);      //Go Forward and Push Robot
        Thread.sleep(2000);

        //robot.move(0.0, 0.0, 100);      //Stop Robot

        robot.move(-0.5, -0.5, 200);      //Go Backward
        //Thread.sleep(200);

        robot.move(0.0, 0.0, 100);      //Stop Robot
        Thread.sleep(200);

        robot.move(-1.0, 1.0, 500);     //Turn Right
        Thread.sleep(300);

        //robot.move(0.0, 0.0, 100);      //Stop Robot
        robot.move(0.5, 0.5, 400);      //Go Forward
        Thread.sleep(1700);

        robot.setClawMove(true ? 0 : 1.0);             //Close Claw
        Thread.sleep(200);

        telemetry.addData("Auto End Time", "%5.2f", getRuntime() );
        telemetry.update();

        // Set the motor to the new power and pause;
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
        //armRotate.setPower(power);
        sleep(CYCLE_MS);
        idle();
        }
}
