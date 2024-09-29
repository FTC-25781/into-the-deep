/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

//package org.firstinspires.ftc.robotcontroller.internal;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
//
///**
// * {@link FtcOpModeRegister} is responsible for registering OpModes for use in an FTC game.
// * @see #register(OpModeManager)
// */
//public class FtcOpModeRegister implements OpModeRegister {
//
//    /**
//     * {@link #register(OpModeManager)} is called by the SDK game in order to register
//     * OpMode classes or instances that will participate in an FTC game.
//     *
//     * There are two mechanisms by which an OpMode may be registered.
//     *
//     *  1) The preferred method is by means of class annotations in the OpMode itself.
//     *  See, for example the class annotations in {@link org.firstinspires.ftc.robotcontroller.external.samples.ConceptNullOp}.
//     *
//     *  2) The other, retired,  method is to modify this {@link #register(OpModeManager)}
//     *  method to include explicit calls to OpModeManager.register().
//     *  This method of modifying this file directly is discouraged, as it
//     *  makes updates to the SDK harder to integrate into your code.
//     *
//     * @param manager the object which contains methods for carrying out OpMode registrations
//     *
//     * @see com.qualcomm.robotcore.eventloop.opmode.TeleOp
//     * @see com.qualcomm.robotcore.eventloop.opmode.Autonomous
//     */
//    public void register(OpModeManager manager) {
//
//        /**
//         * Any manual OpMode class registrations should go here.
//         */
//    }
//}
//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class DrivetrainSubsystem {
//    private DcMotor leftMotor;
//    private DcMotor rightMotor;
//
//    public DrivetrainSubsystem(HardwareMap hardwareMap) {
//        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
//        rightMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse one side
//    }
//
//    public void setPower(double leftPower, double rightPower) {
//        leftMotor.setPower(leftPower);
//        rightMotor.setPower(rightPower);
//    }
//
//    public void stop() {
//        setPower(0, 0);
//    }
//}
//package org.firstinspires.ftc.teamcode.commands;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
//
//public class DriveCommand {
//    private final DrivetrainSubsystem drivetrain;
//    private final Gamepad gamepad;
//
//    public DriveCommand(DrivetrainSubsystem drivetrain, Gamepad gamepad) {
//        this.drivetrain = drivetrain;
//        this.gamepad = gamepad;
//    }
//
//    public void execute() {
//        // Tank drive: Left stick controls the left motor, right stick controls the right motor
//        double leftPower = -gamepad.left_stick_y;
//        double rightPower = -gamepad.right_stick_y;
//
//        drivetrain.setPower(leftPower, rightPower);
//    }
//
//    public void stop() {
//        drivetrain.stop();
//    }
//}
//package org.firstinspires.ftc.teamcode.commands;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
//
//public class DriveCommand {
//    private final DrivetrainSubsystem drivetrain;
//    private final Gamepad gamepad;
//
//    public DriveCommand(DrivetrainSubsystem drivetrain, Gamepad gamepad) {
//        this.drivetrain = drivetrain;
//        this.gamepad = gamepad;
//    }
//
//    public void execute() {
//        // Tank drive: Left stick controls the left motor, right stick controls the right motor
//        double leftPower = -gamepad.left_stick_y;
//        double rightPower = -gamepad.right_stick_y;
//
//        drivetrain.setPower(leftPower, rightPower);
//    }
//
//    public void stop() {
//        drivetrain.stop();
//    }
//}
//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class IntakeSubsystem {
//    private DcMotor intakeMotor;
//
//    public IntakeSubsystem(HardwareMap hardwareMap) {
//        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//    }
//
//    public void runIntake(double power) {
//        intakeMotor.setPower(power);
//    }
//
//    public void stop() {
//        intakeMotor.setPower(0);
//    }
//}
//package org.firstinspires.ftc.teamcode.commands;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//
//public class IntakeCommand {
//    private final IntakeSubsystem intake;
//    private final Gamepad gamepad;
//
//    public IntakeCommand(IntakeSubsystem intake, Gamepad gamepad) {
//        this.intake = intake;
//        this.gamepad = gamepad;
//    }
//
//    public void execute() {
//        if (gamepad.a) {
//            intake.runIntake(1.0);  // Run intake forward
//        } else if (gamepad.b) {
//            intake.runIntake(-1.0); // Run intake backward
//        } else {
//            intake.stop();
//        }
//    }
//
//    public void stop() {
//        intake.stop();
//    }
//}
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;

public class RobotOpMode extends OpMode {
    private DrivetrainSubsystem drivetrain;
    private IntakeSubsystem intake;
    private DriveCommand driveCommand;
    private IntakeCommand intakeCommand;

    @Override
    public void init() {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        // Passing in the gamepad and subsystems to their respective commands
        driveCommand = new DriveCommand(drivetrain, gamepad1);
        intakeCommand = new IntakeCommand(intake, gamepad1);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Execute commands on each loop
        driveCommand.execute();
        intakeCommand.execute();

        telemetry.update();
    }

    @Override
    public void stop() {
        driveCommand.stop();
        intakeCommand.stop();
    }
}
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty

# Key bindings for moving the robot
        move_bindings = {
        'w': (1, 0, 0, 0),  # Forward
    's': (-1, 0, 0, 0), # Backward
    'a': (0, 0, 0, 1),  # Turn left
    'd': (0, 0, 0, -1), # Turn right
    'q': (0, 0, 0, 0),  # Stop
}

        # Function to get keypress from the terminal
def get_key():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def teleop():
        rospy.init_node('teleop_node')  # Initialize the node
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Publisher to /cmd_vel topic
rate = rospy.Rate(10)  # Set rate to 10Hz

    # Create the Twist message object
        twist = Twist()

print("Control the robot using WASD keys (Press 'Ctrl+C' to exit)")

    while not rospy.is_shutdown():
key = get_key()

        if key in move_bindings.keys():
x, y, z, th = move_bindings[key]
twist.linear.x = x  # Move forward/backward
twist.angular.z = th  # Rotate left/right
        else:
twist.linear.x = 0  # Stop movement
twist.angular.z = 0  # Stop rotation

        pub.publish(twist)  # Publish the twist message
        rate.sleep()

        if key == '\x03':  # Detect Ctrl+C (ASCII code for interrupt)
            break

                    if __name__ == "__main__":
settings = termios.tcgetattr(sys.stdin)  # Store terminal settings

    try:
teleop()  # Call the teleop function
except rospy.ROSInterruptException:
pass
    finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore terminal settings
