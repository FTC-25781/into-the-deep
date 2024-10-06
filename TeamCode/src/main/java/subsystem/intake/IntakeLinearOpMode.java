package subsystem.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;

@TeleOp(name = "Intake Control LinearOpMode", group = "LinearOpMode")
public class IntakeLinearOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        IntakeSubsystem intakeSubsystem;
        IntakeCommand intakeCommand;

        // Initialize the subsystem
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);

        // Wait for the game to start
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Run until the end of the match (until stop is pressed)
        while (opModeIsActive()) {
            // Control the intake with gamepad buttons
            if (gamepad1.a) {
                intakeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.IntakeState.START, telemetry);
                intakeCommand.execute();
            } else if (gamepad1.b) {
                intakeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.IntakeState.STOP, telemetry);
                intakeCommand.execute();
            } else if (gamepad1.y) {
                intakeCommand = new IntakeCommand(intakeSubsystem, IntakeCommand.IntakeState.REVERSE, telemetry);
                intakeCommand.execute();
            }

            // Small delay to prevent overloading the CPU
            sleep(100);
        }
    }
}
