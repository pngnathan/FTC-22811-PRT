package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;


/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called org.firstinspires.ftc.teamcode.RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy org.firstinspires.ftc.teamcode.MorrisPOVDrive.java into TeamCode (using Android Studio or OnBotJava) then org.firstinspires.ftc.teamcode.RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 */

/**
 * Mr. Morris:          TO DO:  1) Consider remapping buttons for movement to one joystick and arm rotation to other joystick,
 *                                 or triggers for forward/backward, left stick for turning, right stick for steering
 *                              2) Set preset locations for arm height and maybe gripper limits
 *
 *     COMPLETE, NEEDS TESTING: 1) Finish updating to match RobotHardware file definitions, then delete or comment out @Disabled
 *                              2) Enable Camera live view
 *                              3) Use math to keep wrist turned so that gripper is level with ground (rotate relative to arm rotation)
 *                              4) Add elapsed time tracking and implement better button press delay method. See https://stemrobotics.cs.pdx.edu/node/7262.html
 */

@TeleOp(name="Morris POV Drive", group="Robot")
//@Disabled
public class MorrisPOVDrive extends LinearOpMode {

    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot       = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive;
        double turn;
        double armRotate    = 0;
        double armExtend    = 0;
        double gripper   = 0;
        double aLastTime = 0, bLastTime = 0, xLastTime = 0, yLastTime = 0, rBLastTime = 0, lBLastTime = 0;
        boolean backButtonPressed;
        final double BUTTON_PRESS_DELAY = .075;// seconds, keep track of how long a button has been pressed and allow for a quick press to move a servo a small amount while a long press moves the servo a longer distance.

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play to start OpMode.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Save CPU resources; can resume streaming when needed.
            // This uses a method of tracking the state of the button called latching. See https://stemrobotics.cs.pdx.edu/node/7262.html.
            // It won't toggle until the button is released. This avoids double-presses.
            if (gamepad1.back) {
                if (!gamepad1.back) {
                    robot.toggleStreaming();
                    backButtonPressed = true;
                }
            }
            else backButtonPressed = false;

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            ////Mr. Morris: Alternatively we could use right trigger for forward, left trigger for reverse, left_stick_x for turning
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion. Use org.firstinspires.ftc.teamcode.RobotHardware class
            robot.driveRobot(drive, turn);

            // Use gamepad left & right Bumpers to open and close the gripper claw
            // Use the SERVO constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range

            // Open gripper when right bumper is pressed if it's not already at max, close gripper when left bumper is pressed if it's not already at min
            // Keeps track of how long a button is pressed and moves a small amount for a short press and a larger amount for a long press
            if (gamepad1.right_bumper)
                if (getRuntime() - rBLastTime > BUTTON_PRESS_DELAY)
                {
                    if (gripper < RobotHardware.GRIPPER_MAX)
                        gripper += RobotHardware.GRIPPER_SPEED;
                    rBLastTime = getRuntime();
                }
            else if (gamepad1.left_bumper)
                {
                    if (getRuntime() - lBLastTime > BUTTON_PRESS_DELAY)
                        if (gripper > RobotHardware.GRIPPER_MIN)
                            gripper -= RobotHardware.GRIPPER_SPEED;
                    lBLastTime = getRuntime();
                }
            gripper = Range.clip(gripper, -0.5, 0.5);

            // Move servo to new position.  Use org.firstinspires.ftc.teamcode.RobotHardware class
            robot.setGripperPosition(gripper);

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
            //// Mr. Morris: Consider redefining the arm movements to use a joystick with a,b,x,y buttons reserved for preset positions like in FTC season
            if (gamepad1.y)
                if (getRuntime() - yLastTime > BUTTON_PRESS_DELAY) {
                    if (armRotate < RobotHardware.ARM_ROTATE_MAX)
                        armRotate += RobotHardware.ARM_INCREMENT_DEGREES; // rotate arm up when Y is pressed
                    yLastTime = getRuntime();
                }
            else if (gamepad1.a)
                if (getRuntime() - aLastTime > BUTTON_PRESS_DELAY) {
                    if (armRotate > RobotHardware.ARM_ROTATE_MIN)
                        armRotate -= RobotHardware.ARM_INCREMENT_DEGREES; // rotate arm down when A is pressed
                    aLastTime = getRuntime();
                }
            else armRotate = 0; //updated to match RobotHardware file definitions

            // Use gamepad buttons to extend arm (X) and retract arm (B)
            // Use the MOTOR constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
            if (gamepad1.x && getRuntime() - xLastTime > BUTTON_PRESS_DELAY) {
                if (armExtend < RobotHardware.ARM_EXTEND_MAX)
                    armExtend += RobotHardware.ARM_EXTEND_POWER; // extend when X is pressed
                xLastTime = getRuntime();
                }
            else if (gamepad1.b && getRuntime() - bLastTime > BUTTON_PRESS_DELAY) {
                if (armExtend > RobotHardware.ARM_RETRACT_MAX) {
                    armExtend -= RobotHardware.ARM_RETRACT_POWER; // retract when B is pressed
                }
                bLastTime = getRuntime();
            }
            else armExtend = 0; // don't move if x and b aren't pressed.
            robot.setArmPower(armRotate, armExtend); // send rotation and extension values to robot

            // Move wrist so that it moves when arm rotates to keep gripper parallel to floor
            // e.g. if arm angle is at -30 (30 degrees below forward horizontal), wrist must be 30 (30 degrees above forward horizontal) to keep gripper horizontal
            robot.setWristAngle(-robot.getArmAngle());

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Arm Extend/Retract", "X & B Buttons");
            telemetry.addData("Gripper Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Arm Rotate Power",  "%.2f", armRotate);
            telemetry.addData("Arm Extend Power",  "%.2f", armExtend);
            telemetry.addData("Gripper Position",  "Offset = %.2f", gripper);
            telemetry.update();

            idle(); //share processor with other programs - good to include in any loop structure in a linear OpMode
        }
    }
}
