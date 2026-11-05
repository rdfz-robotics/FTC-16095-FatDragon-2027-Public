package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.PositionHoldController;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit.FiringState;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

public class ShootAssistCommand extends ParallelCommandGroup {
    private final Drive drive;
    private final ShootCommand shooting;
    private final Vision vision;
    private final AutoAimController autoAim;
    private final PositionHoldController positionHold = new PositionHoldController();
    private final GamepadEx gamepadEx;
    private final boolean autoAimEnabled;

    private boolean manualTranslation;
    private double forward;
    private double strafe;
    private double turnPower;

    public ShootAssistCommand(
            Drive drive,
            Shooter shooter,
            Intake intake,
            Transit transit,
            Vision vision,
            AutoAimController autoAim,
            GamepadEx gamepadEx
    ) {
        this(drive, shooter, intake, transit, vision, autoAim, gamepadEx, true);
    }

    public ShootAssistCommand(
            Drive drive,
            Shooter shooter,
            Intake intake,
            Transit transit,
            Vision vision,
            AutoAimController autoAim,
            GamepadEx gamepadEx,
            boolean autoAimEnabled
    ) {
        this.drive = drive;
        this.vision = vision;
        this.autoAim = autoAim;
        this.gamepadEx = gamepadEx;
        this.autoAimEnabled = autoAimEnabled;
        shooting = new ShootCommand(shooter, intake, transit,
                () -> vision.hasValidAimingTarget() ? DistanceUnit.INCH.toCm(
                        vision.getTagCenterHorizontalDistanceInches()) : Double.NaN,
                () -> !autoAimEnabled || (vision.isAimCalibrationReady() && autoAim.isLocked()),
                () -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                        > org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants.triggerThreshold,
                () -> gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER),
                () -> gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER));
        addCommands(shooting, new FunctionalCommand(
                () -> { }, this::applyDrivePower, interrupted -> drive.stop(), () -> false, drive));
    }

    @Override
    public void initialize() {
        autoAim.reset();
        positionHold.reset();
        manualTranslation = false;
        super.initialize();
    }

    @Override
    public void execute() {
        forward = applyDeadband(gamepadEx.getLeftY());
        strafe = applyDeadband(gamepadEx.getLeftX());
        manualTranslation = forward != 0.0 || strafe != 0.0;
        if (!autoAimEnabled || !vision.isAimCalibrationReady() || manualTranslation) {
            positionHold.reset();
        } else {
            positionHold.update(drive.getPose(), drive.isFieldPoseInitialized(),
                    drive.getTranslationSpeedInchesPerSecond());
            forward = positionHold.getForwardPower();
            strafe = positionHold.getStrafePower();
        }

        boolean jamRequested = gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        if (jamRequested) {
            autoAim.reset();
            turnPower = 0.0;
        } else {
            turnPower = autoAimEnabled ? autoAim.calculateTurnPower(
                    vision.hasValidAimingTarget(), vision.getYawErrorDegrees()) : 0.0;
        }
        // Compute this cycle's aim lock before either parallel child executes.
        super.execute();
    }

    private void applyDrivePower() {
        if (autoAimEnabled && !manualTranslation) {
            drive.moveRobot(forward, strafe, turnPower);
        } else {
            // Manual translation stays headless during RT; position hold already returns robot-space powers.
            drive.moveFieldCentric(forward, strafe,
                    autoAimEnabled ? turnPower : applyDeadband(gamepadEx.getRightX()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        autoAim.reset();
        positionHold.reset();
        manualTranslation = false;
    }

    public boolean canFire() { return shooting.canFire(); }
    public boolean canPrepareToFire() { return shooting.canPrepareToFire(); }
    public FiringState getFiringState() { return shooting.getFiringState(); }
    public boolean isManualTranslation() { return manualTranslation; }

    private double applyDeadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
    }
}
