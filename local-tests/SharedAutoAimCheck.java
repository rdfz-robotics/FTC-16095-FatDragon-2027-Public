package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.opmodes.autos.TestAutoCheck;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveState;
import sun.misc.Unsafe;

/** Actual scheduler, composed commands, intake and transit; substitute only hardware/pose/target. */
public final class SharedAutoAimCheck {
    private static int checks;
    private static final Unsafe U = allocator();

    private static Unsafe allocator() {
        try {
            Field f = Unsafe.class.getDeclaredField("theUnsafe");
            f.setAccessible(true);
            return (Unsafe) f.get(null);
        } catch (Exception e) { throw new AssertionError(e); }
    }
    private static Object get(Object object, Class<?> owner, String name) throws Exception {
        Field f = owner.getDeclaredField(name);
        f.setAccessible(true);
        return f.get(object);
    }
    private static void set(Object object, Class<?> owner, String name, Object value) throws Exception {
        Field f = owner.getDeclaredField(name);
        f.setAccessible(true);
        f.set(object, value);
    }
    private static void call(Object object, String name) throws Exception {
        Method m = object.getClass().getDeclaredMethod(name);
        m.setAccessible(true);
        m.invoke(object);
    }
    private static void check(boolean value, String message) {
        checks++;
        if (!value) throw new AssertionError(message);
    }

    private static final class Wheels implements Drivetrain {
        DrivePowers powers = DrivePowers.zero();
        boolean stopped = true;
        public void drive(DrivePowers p, boolean manual) { powers = p; stopped = false; }
        public double maxScaling(DrivePowers a, DrivePowers b) { return 1; }
        public void stop() { stopped = true; powers = DrivePowers.zero(); }
        public void stop(boolean brake) { stop(); }
        public Map<String, Object> debug() { return Collections.emptyMap(); }
        public double interpolateVelocity(double a, double b, double theta) { return 0; }
    }
    private static final class Paths extends Follower {
        Pose current = new Pose(55, 32, Math.PI);
        Velocity speed = Velocity.zero();
        final List<Path> followed = new ArrayList<>();
        final List<Pose> held = new ArrayList<>();
        Paths() { super(null, new Wheels(), null); }
        @Override public Pose pose() { return current; }
        @Override public Velocity velocity() { return speed; }
        @Override public void follow(Path path) { followed.add(path); }
        @Override public void hold(Pose pose) { held.add(pose); }
    }
    private static final class Flywheel extends Shooter {
        boolean ready = true, pitchReady = true, targetSet, rejectDistance;
        int stops;
        Flywheel() { super(null); }
        @Override public boolean setDistanceTarget(double cm) {
            targetSet = Double.isFinite(cm) && !rejectDistance;
            return targetSet;
        }
        @Override public boolean isReady() { return ready; }
        @Override public boolean isPitchReady() { return pitchReady; }
        @Override public void stop() { targetSet = false; stops++; }
        @Override public void periodic() { }
    }
    private static final class Target extends AutoHiveTarget {
        double distance = 100, yaw;
        HiveState state = HiveState.HIGH;
        Target() { super(null, null); }
        @Override public void update(Pose pose) { }
        @Override public double getDistanceCm() { return distance; }
        @Override public double getYawErrorDegrees() { return yaw; }
        @Override public HiveState getState() { return state; }
    }
    private static final class Odometry extends GoBildaPinpointDriver {
        Odometry() { super(null, false); }
        @Override public DeviceStatus getDeviceStatus() { return DeviceStatus.READY; }
    }

    private static final class Fixture {
        final CommandScheduler scheduler;
        final Paths paths = new Paths();
        final Flywheel shooter;
        final Intake intake;
        final Transit transit;
        final Target target;
        boolean located = true;
        AutoShootCommand shot;
        Fixture() throws Exception {
            CommandScheduler.getInstance().reset();
            scheduler = CommandScheduler.getInstance();
            shooter = (Flywheel) U.allocateInstance(Flywheel.class);
            shooter.ready = shooter.pitchReady = true;
            intake = (Intake) U.allocateInstance(Intake.class);
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
            Method factory = TestAutoCheck.class.getDeclaredMethod("transit");
            factory.setAccessible(true);
            transit = (Transit) factory.invoke(null);
            target = (Target) U.allocateInstance(Target.class);
            target.distance = 100;
            target.state = HiveState.HIGH;
            scheduler.registerSubsystem(shooter, intake, transit);
        }
        AutoShootCommand create(boolean high) {
            shot = new AutoShootCommand(paths, shooter, intake, transit, target, () -> located,
                    400, 5000, high);
            return shot;
        }
        void start(boolean high) { scheduler.schedule(create(high)); }
        ShootCommand child() throws Exception {
            return (ShootCommand) get(shot, AutoShootCommand.class, "shooting");
        }
        void settle() throws Exception {
            scheduler.run();
            set(shot, AutoShootCommand.class, "stationarySinceNanos", System.nanoTime() - 300_000_000L);
            scheduler.run();
            scheduler.run();
        }
        void complete() throws Exception {
            check(transit.getFiringState() == Transit.FiringState.FEEDING, "fixture reached actual feeding");
            set(child(), ShootCommand.class, "feedStartedNanos", System.nanoTime() - 500_000_000L);
            scheduler.run();
        }
        void stopped(String reason) {
            intake.periodic();
            transit.periodic();
            check(!shooter.targetSet && intake.getRequestedPower() == 0
                    && transit.getRequestedPower() == 0 && transit.getState() == Transit.TransitState.CLOSE
                    && ((Wheels) paths.drivetrain).stopped && paths.idle(), reason);
        }
        void close() { scheduler.cancelAll(); scheduler.reset(); }
    }

    public static void main(String[] args) throws Exception {
        double stable = DriveConstants.aimLockStableTimeSeconds;
        double reverse = TransitConstants.feedReverseTime;
        boolean hold = DriveConstants.positionHoldCalibrated;
        try {
            DriveConstants.aimLockStableTimeSeconds = 0;
            DriveConstants.positionHoldCalibrated = false;
            TransitConstants.feedReverseTime = 0;
            readinessAndPreheat();
            highAndInvalidTarget();
            timeoutsCancelAndComposition();
            driftGuards();
            routeHandoffs();
            System.out.println("PASS SharedAutoAimCheck: " + checks + " behavioral assertions");
        } finally {
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().reset();
            DriveConstants.aimLockStableTimeSeconds = stable;
            TransitConstants.feedReverseTime = reverse;
            DriveConstants.positionHoldCalibrated = hold;
        }
    }

    private static void readinessAndPreheat() throws Exception {
        Fixture f = new Fixture();
        f.shooter.targetSet = true;
        f.shooter.ready = false;
        f.start(false);
        check(f.shooter.targetSet && f.shooter.stops == 0, "initialization preserves route preheat");
        check(f.shot.getRequirements().size() == 3, "adapter owns exactly the common mechanism requirements");
        boolean refused = false;
        try { f.scheduler.schedule(f.child()); } catch (IllegalArgumentException expected) { refused = true; }
        check(refused, "scheduler rejects independently scheduling the already composed child");
        f.scheduler.run();
        check(f.transit.getState() == Transit.TransitState.CLOSE
                && ((Wheels) f.paths.drivetrain).stopped, "settling does not drive or open the gate");
        f.paths.speed = new Velocity(3, 0, 0);
        set(f.shot, AutoShootCommand.class, "stationarySinceNanos", System.nanoTime() - 300_000_000L);
        f.scheduler.run();
        check(f.transit.getState() == Transit.TransitState.CLOSE
                && get(f.shot, AutoShootCommand.class, "anchor") == null, "a moving chassis cannot establish a shot anchor");
        f.paths.speed = Velocity.zero();
        f.target.yaw = 10;
        f.settle();
        check(((Wheels) f.paths.drivetrain).powers.turn() > 0, "positive target yaw turns Pedro CCW");
        f.target.yaw = -10;
        f.scheduler.run();
        check(((Wheels) f.paths.drivetrain).powers.turn() < 0, "negative target yaw turns Pedro CW");
        f.target.yaw = 0;
        f.scheduler.run();
        f.scheduler.run();
        check(f.transit.getState() == Transit.TransitState.CLOSE, "flywheel readiness independently gates feeding");
        f.shooter.ready = true;
        f.shooter.pitchReady = false;
        f.scheduler.run();
        check(f.transit.getState() == Transit.TransitState.CLOSE, "pitch readiness independently gates feeding");
        set(f.child(), ShootCommand.class, "startedNanos", System.nanoTime() - 1_000_000_000L);
        f.shooter.pitchReady = true;
        f.scheduler.run();
        check(f.transit.getFiringState() == Transit.FiringState.FEEDING
                && f.shot.getFeedElapsedMs() < 100, "preparation time is excluded from feed duration");
        f.shooter.ready = false;
        f.target.yaw = 1;
        f.scheduler.run();
        check(f.transit.getFiringState() == Transit.FiringState.FEEDING, "small aim and flywheel transients do not break committed feeding");
        f.complete();
        check(f.shot.getResult() == ShootCommand.Result.SUCCEEDED && !f.scheduler.isScheduled(f.shot)
                && f.shot.isFinished(), "success remains observable after the composed command ends");
        f.stopped("successful shot stops every output across periodic updates");
        f.close();
    }

    private static void highAndInvalidTarget() throws Exception {
        Fixture f = new Fixture();
        TransitConstants.feedReverseTime = 0.25;
        f.start(true);
        f.settle();
        check(f.transit.getFiringState() == Transit.FiringState.REVERSING, "HIGH permits reverse preparation");
        f.target.state = HiveState.UNKNOWN;
        f.scheduler.run();
        check(f.transit.getState() == Transit.TransitState.CLOSE
                && f.transit.getRequestedPower() == 0 && f.shooter.targetSet,
                "losing HIGH during reverse explicitly cancels preparation while retaining spin-up");
        TransitConstants.feedReverseTime = 0;
        f.target.state = HiveState.HIGH;
        f.scheduler.run();
        check(f.transit.getFiringState() == Transit.FiringState.FEEDING, "HIGH recovery restarts preparation within the same deadline");
        f.target.state = HiveState.UNKNOWN;
        f.scheduler.run();
        check(f.transit.getFiringState() == Transit.FiringState.FEEDING, "HIGH is a start condition; loss after actual feed preserves the original policy");
        f.located = false;
        f.scheduler.run();
        f.stopped("localization loss stops drivetrain and all mechanisms");
        check(f.shot.getResult() == ShootCommand.Result.RUNNING && f.shot.getFeedElapsedMs() == 0,
                "invalid localization resets feed timing without resetting the total deadline");
        f.located = true;
        f.target.distance = Double.NaN;
        f.scheduler.run();
        f.stopped("invalid geometry remains stopped");
        f.target.distance = 100;
        f.shooter.rejectDistance = true;
        f.scheduler.run();
        f.stopped("rejected shooter calibration also blocks drive output");
        f.shooter.rejectDistance = false;
        f.target.state = HiveState.HIGH;
        f.scheduler.run();
        f.scheduler.run();
        check(f.transit.getFiringState() == Transit.FiringState.FEEDING, "valid target and calibration recover in the same command");
        f.complete();
        f.close();
    }

    private static void timeoutsCancelAndComposition() throws Exception {
        Fixture f = new Fixture();
        f.start(true);
        f.target.state = HiveState.UNKNOWN;
        f.settle();
        set(f.child(), ShootCommand.class, "startedNanos", System.nanoTime() - 6_000_000_000L);
        f.scheduler.run();
        check(f.shot.getResult() == ShootCommand.Result.TIMED_OUT && f.shot.hasTimedOut()
                && !f.scheduler.isScheduled(f.shot), "waiting for HIGH terminates at the common total deadline");
        f.stopped("timeout leaves all outputs stopped");
        f.scheduler.schedule(f.shot);
        check(!f.shot.hasTimedOut() && f.shot.getResult() == ShootCommand.Result.RUNNING,
                "rescheduling clears the previous result and timers");
        f.scheduler.cancelAll();
        check(f.shot.getResult() == ShootCommand.Result.INTERRUPTED, "STOP during preparation reports interruption");
        f.stopped("STOP during preparation is persistent");
        f.target.state = HiveState.HIGH;
        f.scheduler.schedule(f.shot);
        f.settle();
        f.scheduler.cancelAll();
        f.stopped("STOP during feeding is persistent");
        f.close();

        f = new Fixture();
        boolean[] companionEnded = {false};
        Command group = new ParallelDeadlineGroup(f.create(false), new FunctionalCommand(
                () -> { }, () -> { }, interrupted -> companionEnded[0] = interrupted, () -> false));
        f.scheduler.schedule(group);
        f.settle();
        f.complete();
        check(!f.scheduler.isScheduled(group) && companionEnded[0],
                "adapter can be a deadline: completion survives FTCLib group end/reset");
        f.stopped("parent composition completion cleans up every output");
        f.close();
    }

    private static void driftGuards() throws Exception {
        for (int mode = 0; mode < 3; mode++) {
            Fixture f = new Fixture();
            if (mode == 0) f.shooter.ready = false;
            f.start(false);
            f.settle();
            if (mode == 0) f.paths.current = new Pose(57, 32, Math.PI);
            if (mode == 1) f.target.yaw = 4;
            if (mode == 2) f.paths.speed = new Velocity(3, 0, 0);
            f.scheduler.run();
            check(f.shot.getResult() == ShootCommand.Result.ABORTED && !f.scheduler.isScheduled(f.shot),
                    "position / feed yaw / feed translation drift aborts with an explicit result: " + mode);
            f.stopped("drift abort stops every output: " + mode);
            f.close();
        }
    }

    private static void routeHandoffs() throws Exception {
        for (String name : new String[]{"RedFourBallCommandAuto", "RedHalfTwoTipTest"}) {
            for (boolean success : new boolean[]{true, false}) {
                Fixture f = new Fixture();
                Class<?> type = Class.forName("org.firstinspires.ftc.teamcode.opmodes.autos." + name);
                Object route = U.allocateInstance(type);
                for (String field : new String[]{"scheduler", "follower", "shooter", "intake", "transit", "target"}) {
                    Object value = field.equals("scheduler") ? f.scheduler : field.equals("follower") ? f.paths
                            : field.equals("shooter") ? f.shooter : field.equals("intake") ? f.intake
                            : field.equals("transit") ? f.transit : f.target;
                    set(route, type, field, value);
                }
                set(route, type, "pinpoint", U.allocateInstance(Odometry.class));
                boolean corner = name.equals("RedFourBallCommandAuto");
                if (corner) {
                    set(route, type, "poseFactory", PoseFactory.degrees());
                    set(route, type, "secondPoint", new Pose(26.03, 110.6426, Math.PI));
                } else {
                    set(route, type, "firstShot", new Pose(53.8928, 31.5834, Math.toRadians(270)));
                }
                call(route, corner ? "prepareShot" : "beginShot");
                f.shot = (AutoShootCommand) get(route, type, "shootCommand");
                check(f.scheduler.isScheduled(f.shot) && f.paths.held.isEmpty(),
                        name + " schedules the aiming adapter without a simultaneous Pedro hold");
                f.settle();
                if (success) {
                    set(f.child(), ShootCommand.class, "feedStartedNanos", System.nanoTime() - 3_000_000_000L);
                } else {
                    set(f.child(), ShootCommand.class, "startedNanos", System.nanoTime() - 14_000_000_000L);
                }
                f.scheduler.run();
                call(route, corner ? "shootAtCorner" : "advanceShot");
                if (success && corner) {
                    check(f.paths.followed.size() == 1 && f.paths.held.isEmpty(),
                            "corner route starts its next path only after successful shot");
                    check(Math.abs(f.paths.followed.get(0).get(0).heading() - f.paths.current.heading()) < 1e-8,
                            "next path uses the measured final shot heading");
                } else if (success) {
                    check(f.paths.followed.isEmpty() && f.paths.held.size() == 1
                                    && Math.abs(f.paths.held.get(0).heading() - Math.toRadians(270)) < 1e-8,
                            "half route restores the exported 270-degree start before the next path");
                } else {
                    check(f.paths.followed.isEmpty() && f.paths.held.isEmpty()
                                    && get(route, type, "stage").toString().equals("DONE"),
                            name + " does not continue after a failed shot");
                    f.stopped(name + " timeout cleanup");
                }
                f.close();
            }
        }
    }
}
