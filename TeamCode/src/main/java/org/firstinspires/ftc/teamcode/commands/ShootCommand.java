package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit.FiringState;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Mechanism-only shooting, shared by TeleOp and autonomous. Distances are centimeters. */
public final class ShootCommand extends CommandBase {
    public enum Result { RUNNING, SUCCEEDED, TIMED_OUT, INTERRUPTED, ABORTED }

    /** Readiness gates opening; these inputs explicitly cancel an unsafe sequence. */
    public enum FeedPermission { ALLOW, WAIT_FOR_START, INVALID_TARGET, ABORT }

    private final Shooter shooter;
    private final Intake intake;
    private final Transit transit;
    private final DoubleSupplier distanceCm;
    private final BooleanSupplier aimReady;
    private final BooleanSupplier intakeRequested;
    private final BooleanSupplier jamRequested;
    private final BooleanSupplier intakeReverseRequested;
    private Supplier<FeedPermission> feedPermission = () -> FeedPermission.ALLOW;
    private boolean preservePreheat;
    private boolean validTarget;
    private long feedDurationMs = -1;
    private long totalTimeoutMs = -1;
    private long startedNanos;
    private long feedStartedNanos = -1;
    private long feedElapsedMs;
    private boolean canPrepareToFire;
    private Result result = Result.INTERRUPTED;

    /** Runs until cancelled. The owning command provides optional operator controls. */
    public ShootCommand(Shooter shooter, Intake intake, Transit transit,
                        DoubleSupplier distanceCm, BooleanSupplier aimReady,
                        BooleanSupplier intakeRequested, BooleanSupplier jamRequested,
                        BooleanSupplier intakeReverseRequested) {
        this.shooter = Objects.requireNonNull(shooter);
        this.intake = Objects.requireNonNull(intake);
        this.transit = Objects.requireNonNull(transit);
        this.distanceCm = Objects.requireNonNull(distanceCm);
        this.aimReady = Objects.requireNonNull(aimReady);
        this.intakeRequested = Objects.requireNonNull(intakeRequested);
        this.jamRequested = Objects.requireNonNull(jamRequested);
        this.intakeReverseRequested = Objects.requireNonNull(intakeReverseRequested);
        addRequirements(shooter, intake, transit);
    }

    /** Feed time begins when Transit enters FEEDING. The total timeout includes preparation. */
    public ShootCommand(Shooter shooter, Intake intake, Transit transit,
                        DoubleSupplier distanceCm, BooleanSupplier aimReady,
                        long feedDurationMs, long totalTimeoutMs) {
        this(shooter, intake, transit, distanceCm, aimReady,
                () -> false, () -> false, () -> false);
        if (feedDurationMs <= 0 || totalTimeoutMs <= feedDurationMs) {
            throw new IllegalArgumentException("Require 0 < feed duration < total timeout");
        }
        this.feedDurationMs = feedDurationMs;
        this.totalTimeoutMs = totalTimeoutMs;
    }

    /** Autonomous adapter supplies safety decisions; only this command writes the mechanisms. */
    public ShootCommand(Shooter shooter, Intake intake, Transit transit,
                        DoubleSupplier distanceCm, BooleanSupplier aimReady,
                        Supplier<FeedPermission> feedPermission, boolean preservePreheat,
                        long feedDurationMs, long totalTimeoutMs) {
        this(shooter, intake, transit, distanceCm, aimReady, feedDurationMs, totalTimeoutMs);
        this.feedPermission = Objects.requireNonNull(feedPermission);
        this.preservePreheat = preservePreheat;
    }

    @Override
    public void initialize() {
        startedNanos = System.nanoTime();
        feedElapsedMs = 0;
        result = Result.RUNNING;
        validTarget = false;
        if (!preservePreheat) shooter.stop();
        if (totalTimeoutMs > 0) intake.stop();
        resetFiringSequence(FiringState.WAITING);
    }

    @Override
    public void execute() {
        if (result != Result.RUNNING) return;
        if (totalTimeoutMs > 0 && elapsedMs(startedNanos) >= totalTimeoutMs) {
            result = Result.TIMED_OUT;
            return;
        }
        FeedPermission permission = feedPermission.get();
        if (permission == FeedPermission.ABORT) {
            result = Result.ABORTED;
            return;
        }
        if (permission == null || permission == FeedPermission.INVALID_TARGET) {
            validTarget = false;
            shooter.stop();
            intake.stop();
            resetFiringSequence(FiringState.WAITING);
            return;
        }
        boolean jam = jamRequested.getAsBoolean();
        boolean reverseIntake = intakeReverseRequested.getAsBoolean();
        intake.setReverseRequested(reverseIntake);
        intake.updateInputs(intakeRequested.getAsBoolean(), jam, true);
        if (jam) {
            validTarget = false;
            shooter.stop();
            resetFiringSequence(FiringState.WAITING);
            transit.reverse();
            return;
        }

        validTarget = shooter.setDistanceTarget(distanceCm.getAsDouble());
        if (!validTarget) {
            resetFiringSequence(FiringState.WAITING);
            if (totalTimeoutMs > 0) intake.stop();
        } else if (permission == FeedPermission.WAIT_FOR_START && !canFire()) {
            // HIGH may disappear during REVERSING. requestFeed(false) alone
            // would keep that committed sequence running, so cancel explicitly.
            resetFiringSequence(FiringState.WAITING);
        } else {
            canPrepareToFire = aimReady.getAsBoolean()
                    && shooter.isReady() && shooter.isPitchReady();
            transit.requestFeed(canPrepareToFire);
        }
        // Preserve the manual left-bumper unjamming behavior.
        if (reverseIntake) transit.forwardForIntakeReverse();

        if (canFire()) {
            if (feedStartedNanos < 0) feedStartedNanos = System.nanoTime();
            feedElapsedMs = elapsedMs(feedStartedNanos);
            if (feedDurationMs > 0 && feedElapsedMs >= feedDurationMs) {
                result = Result.SUCCEEDED;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return result != Result.RUNNING;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) result = Result.INTERRUPTED;
        long lastFeedElapsedMs = feedElapsedMs;
        resetFiringSequence(FiringState.IDLE);
        feedElapsedMs = lastFeedElapsedMs;
        shooter.stop();
        intake.stop();
    }

    public boolean canFire() { return transit.getFiringState() == FiringState.FEEDING; }
    public boolean canPrepareToFire() { return canPrepareToFire; }
    public FiringState getFiringState() { return transit.getFiringState(); }
    public Result getResult() { return result; }
    public long getFeedElapsedMs() { return feedElapsedMs; }
    public boolean hasValidTarget() { return validTarget; }

    private void resetFiringSequence(FiringState next) {
        if (transit.getState() == Transit.TransitState.OPEN) intake.clearBallCount();
        canPrepareToFire = false;
        feedStartedNanos = -1;
        feedElapsedMs = 0;
        transit.cancelFeeding();
        if (next != FiringState.IDLE) transit.requestFeed(false);
    }

    private static long elapsedMs(long startedNanos) {
        return (System.nanoTime() - startedNanos) / 1_000_000L;
    }
}
