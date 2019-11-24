{\rtf1\ansi\ansicpg1252\cocoartf1504\cocoasubrtf840
{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs24 \cf0 /* Copyright (c) 2017 FIRST. All rights reserved.\
 *\
 * Redistribution and use in source and binary forms, with or without modification,\
 * are permitted (subject to the limitations in the disclaimer below) provided that\
 * the following conditions are met:\
 *\
 * Redistributions of source code must retain the above copyright notice, this list\
 * of conditions and the following disclaimer.\
 *\
 * Redistributions in binary form must reproduce the above copyright notice, this\
 * list of conditions and the following disclaimer in the documentation and/or\
 * other materials provided with the distribution.\
 *\
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or\
 * promote products derived from this software without specific prior written permission.\
 *\
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS\
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,\
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE\
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE\
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL\
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR\
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER\
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,\
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE\
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\
 */\
\
package org.firstinspires.ftc.teamcode.samples;\
\
import com.qualcomm.robotcore.eventloop.opmode.Disabled;\
import org.firstinspires.ftc.robotcore.external.navigation.Position;\
import com.qualcomm.robotcore.hardware.DistanceSensor;\
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;\
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;\
import com.qualcomm.robotcore.eventloop.opmode.OpMode;\
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;\
import com.qualcomm.robotcore.util.Range;\
\
/**\
 * This file provides basic Telop driving for a Pushbot robot.\
 * The code is structured as an Iterative OpMode\
 *\
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.\
 * All device access is managed through the HardwarePushbot class.\
 *\
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot\
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.\
 * It also opens and closes the claws slowly using the left and right Bumper buttons.\
 *\
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.\
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list\
 */\
\
//\
// Rev Expansion Hub Motor designations\
// 0 - front left ( left drive )\
// 1 - back left  ( aux left drive )\
// 2 - front right ( aux right drive )\
// 3 - back right  ( right drive )\
// \
@TeleOp(name="Concept: Teleop Tank 2", group="Concept")\
//@Disabled\
public class PushbotTeleopTank_Iterative_Test9 extends OpMode\{\
\
    /* Declare OpMode members. */\
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware\
    double          clawOffset  = 0.0 ;                  // Servo mid position\
    double          leftClawPosition;\
    final double    CLAW_SPEED  = 0.005 ;                 // sets rate to move servo\
    \
    double          gripOffset  = 0.0 ;                  // Servo mid position\
    double          gripPosition;\
\
    /*\
     * Code to run ONCE when the driver hits INIT\
     */\
    @Override\
    public void init() \{\
        /* Initialize the hardware variables.\
         * The init() method of the hardware class does all the work here\
         */\
        robot.init(hardwareMap);\
\
        // Send telemetry message to signify robot waiting;\
        telemetry.addData("Say", "Hello Driver");    //\
    \}\
\
    /*\
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY\
     */\
    @Override\
    public void init_loop() \{\
    \}\
\
    /*\
     * Code to run ONCE when the driver hits PLAY\
     */\
    @Override\
    public void start() \{\
    \}\
\
    /*\
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP\
     */\
    @Override\
    public void loop() \{\
        double left;\
        double right;\
        double armPower;\
        double triggerRightPosition;\
        double triggerLeftPosition;\
\
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)\
        left = -gamepad1.left_stick_y;\
        right = -gamepad1.right_stick_y;\
\
        \
        if (gamepad1.left_bumper) \{\
            robot.leftDrive.setPower(-1);\
            robot.rightDrive.setPower(1);\
            robot.auxLeftDrive.setPower(1);\
            robot.auxRightDrive.setPower(1);\
        \} else if ( gamepad1.right_bumper) \{\
            robot.leftDrive.setPower(1);\
            robot.rightDrive.setPower(-1);\
            robot.auxLeftDrive.setPower(-1);\
            robot.auxRightDrive.setPower(-1);\
        \} else if (gamepad1.dpad_left) \{\
            robot.leftDrive.setPower(-1);\
            robot.rightDrive.setPower(-1);\
            robot.auxLeftDrive.setPower(1);\
            robot.auxRightDrive.setPower(1);\
        \} else if (gamepad1.dpad_right) \{\
            robot.leftDrive.setPower(1);\
            robot.rightDrive.setPower(1);\
            robot.auxLeftDrive.setPower(-1);\
            robot.auxRightDrive.setPower(-1);\
        \} else if (gamepad2.dpad_up) \{\
            gripOffset += CLAW_SPEED;\
        \} else if (gamepad2.dpad_down) \{\
            gripOffset -= CLAW_SPEED;\
        \} else \{\
            robot.leftDrive.setPower(left);\
            robot.rightDrive.setPower(right);\
            robot.auxLeftDrive.setPower(left);\
            robot.auxRightDrive.setPower(-right);\
        \}\
        \
        armPower = gamepad2.right_stick_y/3;\
        robot.linearActuator.setPower(armPower);\
        telemetry.addData("armPower","%.2f", armPower);\
        // Use gamepad left & right Bumpers to open and close the claw\
//        if (gamepad1.right_bumper)\
//            clawOffset += CLAW_SPEED;\
//        else if (gamepad1.left_bumper)\
//            clawOffset -= CLAW_SPEED;\
\
        // Move both servos to new position.  Assume servos are mirror image of each other.\
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);\
//        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);\
//        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);\
//        robot.auxArm.setPosition(robot.MID_SERVO + clawOffset);\
        if (gamepad1.x) \{\
            telemetry.addData("Reset leftClaw servo", "Offset = %.2f", 1.0);\
            //robot.leftClaw.setPosition(0.0);\
        \}\
        // Use gamepad buttons to move the arm up (Y) and down (A)\
        if (gamepad1.y)\
            //robot.leftArm.setPower(robot.ARM_UP_POWER);\
            clawOffset += CLAW_SPEED;\
\
        else if (gamepad1.a)\
            //robot.leftArm.setPower(robot.ARM_DOWN_POWER);\
            clawOffset -= CLAW_SPEED;\
        \
        else\
            //robot.leftArm.setPower(0.0);\
            clawOffset = clawOffset;\
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);\
        leftClawPosition=0.83 + clawOffset;\
        leftClawPosition = Range.clip(leftClawPosition, 0.83, 1.0);\
        //robot.leftClaw.setPosition(leftClawPosition);\
\
        //robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);\
\
        //gripPosition = robot.MID_SERVO + gripOffset;\
        gripPosition = gripOffset;\
        gripPosition = Range.clip(gripPosition, -1.0, 1.0);\
        //robot.auxArm.setPosition(gripPosition);\
\
        triggerLeftPosition = gamepad2.left_trigger;\
        triggerRightPosition = gamepad2.right_trigger;\
        \
        //robot.leftGrip.setPosition(-gripPosition);\
        //robot.rightGrip.setPosition(gripPosition);\
        \
        robot.leftGrip.setPosition(triggerLeftPosition);\
        robot.rightGrip.setPosition(triggerRightPosition);\
        \
        // Send telemetry message to signify robot running;\
        \
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);\
        telemetry.addData("position:", "%.2f", leftClawPosition);\
        telemetry.addData("gripOffset",  "Offset = %.2f", gripOffset);\
        telemetry.addData("left gripPosition:", "%.2f", -gripPosition);\
        telemetry.addData("right gripPosition:", "%.2f", gripPosition);\
        telemetry.addData("left",  "%.2f", left);\
        telemetry.addData("right", "%.2f", right);\
        \
        telemetry.addData("Right trigger","%.2f", gamepad2.right_trigger);\
        telemetry.addData("Left trigger","%.2f", gamepad2.left_trigger);\
        \
        // you can also cast this to a Rev2mDistanceSensor if you want to use added\
        // methods associated with the Rev2mDistanceSensor class.\
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)robot.sensorRange;\
\
        telemetry.addData(">>", "Press start to continue");\
        telemetry.update();\
\
        //waitForStart();\
        //while(opModeIsActive()) \{\
        // generic DistanceSensor methods.\
        //telemetry.addData("deviceName",robot.sensorRange.getDeviceName() );\
        //telemetry.addData("range", String.format("%.01f mm", robot.sensorRange.getDistance(DistanceUnit.MM)));\
        //telemetry.addData("range", String.format("%.01f cm", robot.sensorRange.getDistance(DistanceUnit.CM)));\
        //telemetry.addData("range", String.format("%.01f m", robot.sensorRange.getDistance(DistanceUnit.METER)));\
        //telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));\
\
        // Rev2mDistanceSensor specific methods.\
        //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));\
        //telemetry.addData("did time out 1", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));\
\
        //telemetry.update();\
        \
        // (robot.sensorRange.getDistance(DistanceUnit.METER) < 0.2) \
            //continue;\
        //else \
            //continue;\
                \
        //\}\
                \
                /* code */\
            /* code */\
    \}\
        \
\
    /*\
     * Code to run ONCE after the driver hits STOP\
     */\
    @Override\
    public void stop() \{\
    \}\
\}\
}