package org.firstinspires.ftc.teamcode;/* Copyright (c) 2022 FIRST. All rights reserved.
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

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;

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
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear OpMode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the org.firstinspires.ftc.teamcode.RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, select this sample, and select TeleOp.
 *  Also add another new file named org.firstinspires.ftc.teamcode.RobotHardware.java, select the sample with that name, and select Not an OpMode.
 */

////Mr. Morris: TO DO:  1) Finish updating to match RobotHardware file definitions, then delete or comment out @Disabled
////                    2) Consider remapping buttons for movement to one joystick and arm rotation to other joystick.
////                    3) Set preset locations for arm height and maybe gripper limits
////                    4) Use math to keep wrist turned so that gripper is level with ground (rotate relative to arm rotation)
////                    5) Enable Camera live view

@TeleOp(name="Morris POV Drive", group="Robot")
//@Disabled
public class MorrisPOVDrive extends LinearOpMode {

    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot       = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive        = 0;
        double turn         = 0;
        double armRotate    = 0; //updated to match RobotHardware file definitions
        double armExtend    = 0; //added to match RobotHardware file definitions
        double gripperOffset   = 0; //TO DO: update to match RobotHardware file definitions

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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
            if (gamepad1.right_bumper)
                gripperOffset += robot.GRIPPER_SPEED; //updated to match RobotHardware file definitions, should be called GRIPPER_SPEED and WRIST_SPEED
            else if (gamepad1.left_bumper)
                gripperOffset -= robot.GRIPPER_SPEED; //updated to match RobotHardware file definitions, should be called GRIPPER_SPEED and WRIST_SPEED
            gripperOffset = Range.clip(gripperOffset, -0.5, 0.5);

            // Move both servos to new position.  Use org.firstinspires.ftc.teamcode.RobotHardware class
            robot.setGripperPositions(gripperOffset); //updated to match RobotHardware file definitions

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
            //// Mr. Morris: Consider redefining the arm movements to use a joystick or triggers with a,b,x,y buttons reserved for preset positions

            if (gamepad1.y)
                armRotate = robot.ARM_UP_POWER; //updated to match RobotHardware file definitions
            else if (gamepad1.a)
                armRotate = robot.ARM_DOWN_POWER; //updated to match RobotHardware file definitions
            else
                armRotate = 0; //updated to match RobotHardware file definitions

            if (gamepad1.x)
                armExtend = robot.ARM_EXTEND_POWER; //updated to match RobotHardware file definitions
            else if (gamepad1.b)
                armExtend = robot.ARM_RETRACT_POWER; //updated to match RobotHardware file definitions
            else
                armExtend = 0; //updated to match RobotHardware file definitions
            robot.setArmPower(armRotate, armExtend); //updated to match RobotHardware file definitions

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers"); //TO DO: update to match RobotHardware file definitions
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Arm Power",  "%.2f", arm); //TO DO: update to match RobotHardware file definitions
            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset); //TO DO: update to match RobotHardware file definitions
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
