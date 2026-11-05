package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

import java.util.Objects;
import java.util.function.BooleanSupplier;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final Transit transit;
    private final BooleanSupplier intakeRequested;
    private final BooleanSupplier jamRequested;
    private final BooleanSupplier reverseRequested;

    public IntakeCommand(Intake intake, Transit transit, GamepadEx gamepadEx) {
        this(intake, transit,
                () -> gamepadEx.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                        > TeleOpConstants.triggerThreshold,
                () -> gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER),
                () -> gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER));
    }

    /** Collect until cancelled, or use withTimeout() / a path deadline in autonomous. */
    public IntakeCommand(Intake intake, Transit transit) {
        this(intake, transit, () -> true, () -> false, () -> false);
    }

    public IntakeCommand(Intake intake, Transit transit, BooleanSupplier intakeRequested,
                         BooleanSupplier jamRequested, BooleanSupplier reverseRequested) {
        this.intake = Objects.requireNonNull(intake);
        this.transit = Objects.requireNonNull(transit);
        this.intakeRequested = Objects.requireNonNull(intakeRequested);
        this.jamRequested = Objects.requireNonNull(jamRequested);
        this.reverseRequested = Objects.requireNonNull(reverseRequested);
        addRequirements(intake, transit);
    }

    @Override
    public void initialize() { updateIntakeRequest(); }

    @Override
    public void execute() { updateIntakeRequest(); }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        transit.cancelFeeding();
    }

    private void updateIntakeRequest() {
        boolean collect = intakeRequested.getAsBoolean();
        boolean jam = jamRequested.getAsBoolean();
        boolean reverse = reverseRequested.getAsBoolean();
        intake.setReverseRequested(reverse);
        intake.updateInputs(collect, jam);
        transit.updateIntakeRequest(collect, jam, reverse);
    }
}
