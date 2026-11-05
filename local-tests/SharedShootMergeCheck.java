package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.opmodes.autos.TestAutoCheck;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;
import sun.misc.Unsafe;

/** Real scheduler/commands/intake/transit; fake sensors and flywheel readiness, no hardware. */
public final class SharedShootMergeCheck {
    private static int checks;
    private static final Unsafe U = allocator();

    private static Unsafe allocator() {
        try {
            Field f = Unsafe.class.getDeclaredField("theUnsafe");
            f.setAccessible(true);
            return (Unsafe) f.get(null);
        } catch (Exception e) { throw new AssertionError(e); }
    }

    private static void set(Object object, Class<?> owner, String name, Object value) throws Exception {
        Field f = owner.getDeclaredField(name);
        f.setAccessible(true);
        f.set(object, value);
    }

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static final class Flywheel extends Shooter {
        boolean ready;
        boolean stopped;
        Flywheel() { super(null); }
        @Override public boolean setDistanceTarget(double cm) {
            stopped = !Double.isFinite(cm);
            return !stopped;
        }
        @Override public boolean isReady() { return ready; }
        @Override public boolean isPitchReady() { return true; }
        @Override public void stop() { stopped = true; }
        @Override public void periodic() { }
    }

    public static void main(String[] args) throws Exception {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        Flywheel shooter = (Flywheel) U.allocateInstance(Flywheel.class);
        Intake intake = (Intake) U.allocateInstance(Intake.class);
        set(intake, Intake.class, "intakeMotor", Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (p, m, a) -> {
                    if (m.getName().equals("setPower")) return null;
                    throw new AssertionError(m);
                }));
        set(intake, Intake.class, "colorDistanceSensor", Proxy.newProxyInstance(
                DistanceSensor.class.getClassLoader(), new Class<?>[]{DistanceSensor.class}, (p, m, a) -> {
                    if (m.getName().equals("getDistance")) return 10.0;
                    throw new AssertionError(m);
                }));
        Method fixture = TestAutoCheck.class.getDeclaredMethod("transit");
        fixture.setAccessible(true);
        Transit transit = (Transit) fixture.invoke(null);
        TransitConstants.feedReverseTime = 0;
        scheduler.registerSubsystem(shooter, intake, transit);
        boolean[] aimed = {false};
        double[] distance = {100};
        ShootCommand shot = new ShootCommand(shooter, intake, transit,
                () -> distance[0], () -> aimed[0], 50, 5000);
        check(shot.getRequirements().size() == 3 && shot.getRequirements().contains(shooter)
                && shot.getRequirements().contains(intake) && shot.getRequirements().contains(transit),
                "shared shot owns mechanisms only, leaving Pedro free to hold the chassis");
        scheduler.schedule(shot);
        scheduler.run();
        check(!shot.canFire(), "unready shooter and aim block feeding");
        shooter.ready = true;
        scheduler.run();
        check(!shot.canFire(), "aim must independently be ready");
        aimed[0] = true;
        scheduler.run();
        check(shot.canFire(), "ready mechanism and aim start feeding");
        distance[0] = Double.NaN;
        scheduler.run();
        check(!shot.canFire() && transit.getState() == Transit.TransitState.CLOSE,
                "invalid distance cancels an already committed feed");
        check(shot.getFeedElapsedMs() == 0, "invalid target resets continuous feed timing");
        distance[0] = 100;
        scheduler.run();
        check(shot.canFire(), "valid target can recover within total timeout");
        set(shot, ShootCommand.class, "feedStartedNanos", System.nanoTime() - 100_000_000L);
        scheduler.run();
        check(!scheduler.isScheduled(shot) && shot.getResult() == ShootCommand.Result.SUCCEEDED,
                "completed feed duration ends the command through the scheduler");
        stopped(shooter, intake, transit);
        scheduler.schedule(shot);
        check(shot.getResult() == ShootCommand.Result.RUNNING, "rescheduling resets the result");
        set(shot, ShootCommand.class, "startedNanos", System.nanoTime() - 6_000_000_000L);
        scheduler.run();
        check(!scheduler.isScheduled(shot) && shot.getResult() == ShootCommand.Result.TIMED_OUT,
                "total deadline exits with a distinct timeout result");
        stopped(shooter, intake, transit);
        scheduler.schedule(shot);
        scheduler.run();
        scheduler.cancel(shot);
        check(shot.getResult() == ShootCommand.Result.INTERRUPTED, "cancellation reports interruption");
        stopped(shooter, intake, transit);
        IntakeCommand collect = new IntakeCommand(intake, transit);
        scheduler.schedule(collect);
        scheduler.run();
        check(intake.getRequestedPower() == IntakeConstants.activePower, "autonomous intake needs no gamepad");
        set(intake, Intake.class, "ballCount", IntakeConstants.maxBallCount);
        scheduler.cancel(collect);
        scheduler.run();
        check(intake.getRequestedPower() == 0 && transit.getRequestedPower() == 0,
                "cancelled full intake stays stopped across periodic updates");
        checkDistances();
        scheduler.cancelAll();
        scheduler.reset();
        System.out.println("PASS SharedShootMergeCheck: " + checks + " integration assertions");
    }

    private static void stopped(Flywheel shooter, Intake intake, Transit transit) {
        intake.periodic();
        transit.periodic();
        check(shooter.stopped && intake.getRequestedPower() == 0 && transit.getRequestedPower() == 0
                        && transit.getState() == Transit.TransitState.CLOSE,
                "completion/cancel stops mechanisms and closes gate across periodic updates");
    }

    private static void checkDistances() throws Exception {
        Pose pose = new Pose(55, 110, Math.PI);
        double[] opening = VisionConstants.cellTargetFieldXYInches[
                CellCluster.RED_OPPOSITE_AUDIENCE.getTuningIndex()];
        double expected = 2.54 * Math.hypot(opening[0] - (pose.x() - 72), opening[1] - (pose.y() - 72));
        for (String name : new String[]{"RedFourBallCornerTest"}) {
            Class<?> type = Class.forName("org.firstinspires.ftc.teamcode.opmodes.autos." + name);
            Object auto = U.allocateInstance(type);
            set(auto, type, "shootHoldPose", pose);
            Method distance = type.getDeclaredMethod("shotDistanceCm");
            distance.setAccessible(true);
            check(Math.abs((double) distance.invoke(auto) - expected) < 1e-9,
                    name + " matches the CELL-opening calibration, not the old tag centroid");
        }
    }
}
