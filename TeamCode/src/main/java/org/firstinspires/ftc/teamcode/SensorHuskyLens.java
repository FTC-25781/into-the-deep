package org.firstinspires.ftc.teamcode;

// Import statements for necessary libraries
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Sensor: HuskyLens Servo Control", group = "Sensor")
public class SensorHuskyLens extends LinearOpMode {

    private final int READ_PERIOD = 100;  // Time between reads from the HuskyLens sensor (in milliseconds)
    private HuskyLens huskyLens;
    private final ElapsedTime nClock = new ElapsedTime();  // Timer to measure elapsed time
    private Servo cameraServo;  // Servo controlling the camera
    private Servo clawDirection;  // Servo controlling the claw orientation

    private final int SCREEN_WIDTH = 320;  // Screen width in pixels
    private final int SCREEN_HEIGHT = 240;  // Screen height in pixels
    private final int CROSSHAIR_X = SCREEN_WIDTH / 2;  // X-coordinate of the crosshair (center of screen)
    private final int CROSSHAIR_Y = SCREEN_HEIGHT / 2;  // Y-coordinate of the crosshair (center of screen)

    private boolean blockLocked = false;
    private long actionStartTime = 0;
    private static final long ACTION_DURATION = 5000;  // Time in milliseconds before resetting servos (10 seconds)
    private static final long COOLDOWN_PERIOD = 3000;  // 3 second cooldown to prevent immediate flip back

    @Override
    public void runOpMode() {
        // Initialize the hardware map for servos and sensors
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        clawDirection = hardwareMap.get(Servo.class, "clawDirection");

        // Recalibrate both servos to 0.0 at the start of the program
        recalibrateServos();

        // Deadline for sensor read rate limit
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) continue;
            rateLimit.reset();

            // Read detected blocks from the HuskyLens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length == 0) continue;  // If no blocks detected, continue

            // Lock onto the first detected block if none is locked
            if (!blockLocked && blocks.length > 0) {
                blockLocked = true;
                actionStartTime = System.currentTimeMillis();
            }

            // If a block is locked, rotate camera to 180 degrees and move the claw
            if (blockLocked) {
                rotateServoToAngle(cameraServo, 180);  // Rotate camera to 180 degrees

                // Calculate claw position based on block's angle
                double clawPosition = calculateClawPositionFromAngle(blocks[0].x);
                clawDirection.setPosition(clawPosition);  // Move claw to match block orientation

                long currentTime = System.currentTimeMillis();
                if (currentTime - actionStartTime >= ACTION_DURATION) {
                    // If action duration passed, reset servos and enter cooldown
                    resetAfterAction();
                    sleep(COOLDOWN_PERIOD);
                    blockLocked = false;  // Unlock to allow new detection
                }
            }

            telemetry.update();
        }
    }

    // Reset servos to 0.0 position
    private void recalibrateServos() {
        rotateServoToAngle(cameraServo, 0);
        clawDirection.setPosition(0.0);  // Recalibrate claw to 0.0
    }

    // Rotate servo to specific angle (from 0 to 180 degrees)
    private void rotateServoToAngle(Servo servo, double angle) {
        double position = angle / 300.0;
        position = Math.max(0.0, Math.min(1.0, position));  // Clamp between 0.0 and 1.0
        servo.setPosition(position);
    }

    // Calculate the claw position based on the block's X position to match its orientation
    private double calculateClawPositionFromAngle(int xPosition) {
        // Here, we're assuming the X position corresponds to a degree angle
        double angle = (CROSSHAIR_X - xPosition) * (180.0 / SCREEN_WIDTH); // Convert pixel distance to angle

        // Map angle to a servo position (0.0 to 1.0)
        double clawPosition = (angle + 180.0) / 360.0;  // Normalize angle to a range of 0 to 1
        return Math.max(0.0, Math.min(1.0, clawPosition));  // Ensure it's clamped between 0.0 and 1.0
    }

    // Reset camera and claw after action duration
    private void resetAfterAction() {
        telemetry.addData("Recalibration", "Rotating servos back to 0...");
        rotateServoToAngle(cameraServo, 0);
        clawDirection.setPosition(0.0);
        telemetry.update();
    }
}
