package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {

    private Servo intakeMotor; // Motor controlling the intake
    private Telemetry telemetry;

    // Constructor for initializing the subsystem
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(Servo.class, "intakeMotor");

        // Set motor direction and behavior
        // intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        // intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Method to start the intake
    public void startIntake() {
        intakeMotor.setPosition(1.0); // Set motor power to full
        telemetry.addData("Intake", "Running");
        telemetry.update();
    }

    // Method to stop the intake
    public void stopIntake() {
        intakeMotor.setPosition(0.0); // Set motor power to 0
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    // Method to reverse the intake
    public void reverseIntake() {
        intakeMotor.setPosition(0.5); // Set motor power to reverse
        telemetry.addData("Intake", "Reversing");
        telemetry.update();
    }
}
