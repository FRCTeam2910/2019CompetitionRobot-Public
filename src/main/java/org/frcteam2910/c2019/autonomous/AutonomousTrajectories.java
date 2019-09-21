package org.frcteam2910.c2019.autonomous;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.Side;

public class AutonomousTrajectories {
    private static final double PICKUP_ENDING_VELOCITY = 10.0 * 12.0;

    private final Trajectory hab1ToCargoSideNearTrajectoryLeft;
    private final Trajectory hab1ToCargoSideNearTrajectoryRight;

    private final Trajectory hab2ToCargoSideNearTrajectoryLeft;
    private final Trajectory hab2ToCargoSideNearTrajectoryRight;

    private final Trajectory cargoSideNearToLoadingStationTrajectoryLeft;
    private final Trajectory cargoSideNearToLoadingStationTrajectoryRight;

    private final Trajectory loadingStationToCargoSideMidTrajectoryLeft;
    private final Trajectory loadingStationToCargoSideMidTrajectoryRight;

    private final Trajectory cargoSideMidToLoadingStationTrajectoryLeft;
    private final Trajectory cargoSideMidToLoadingStationTrajectoryRight;

    private final Trajectory loadingStationToCargoSideFarTrajectoryLeft;
    private final Trajectory loadingStationToCargoSideFarTrajectoryRight;

    private final Trajectory loadingStationToRocketNearTrajectoryLeft;
    private final Trajectory loadingStationToRocketNearTrajectoryRight;

    private final Trajectory rocketNearToLoadingStationTrajectoryLeft;
    private final Trajectory rocketNearToLoadingStationTrajectoryRight;

    private final Trajectory loadingStationToRocketFarTrajectoryLeft;
    private final Trajectory loadingStationToRocketFarTrajectoryRight;

    public AutonomousTrajectories(ITrajectoryConstraint... constraints) {
        // <editor-fold desc="Hab to Cargo Ship Side Near">
        Path habToCargoSideNearPathLeft = new Path(Rotation2.fromDegrees(90.0));
        habToCargoSideNearPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(207.0, 12.13)));
        habToCargoSideNearPathLeft.subdivide(8);
        Path habToCargoSideNearPathRight = habToCargoSideNearPathLeft.mirror();
        hab1ToCargoSideNearTrajectoryLeft = new Trajectory(habToCargoSideNearPathLeft, constraints);
        hab1ToCargoSideNearTrajectoryRight = new Trajectory(habToCargoSideNearPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Hab 2 to Cargo Ship Side Near">
        Path hab2ToCargoSideNearPathLeft = new Path(Rotation2.fromDegrees(90.0));
        hab2ToCargoSideNearPathLeft.addSegment(new PathLineSegment(
                new Vector2(0.0, 0.0),
                new Vector2(265.5, 22.13)
        ), Rotation2.fromDegrees(90.0));
        hab2ToCargoSideNearPathLeft.subdivide(8);
        Path hab2ToCargoSideNearPathRight = hab2ToCargoSideNearPathLeft.mirror();
        hab2ToCargoSideNearTrajectoryLeft = new Trajectory(hab2ToCargoSideNearPathLeft, constraints);
        hab2ToCargoSideNearTrajectoryRight = new Trajectory(hab2ToCargoSideNearPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Cargo Ship Side Near to Loading Station">
        Path cargoSideNearToLoadingStationPathLeft = new Path(Rotation2.fromDegrees(90.0));
        cargoSideNearToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(-161.25, 57.13)
                ), Rotation2.fromDegrees(0.0));
//        cargoSideNearToLoadingStationPathLeft.addSegment(
//                new PathArcSegment(
//                        new Vector2(-3.11, 11.59),
//                        new Vector2(-38.84, 45.98),
//                        new Vector2(-49.47, -0.83)
//                ));
//        cargoSideNearToLoadingStationPathLeft.addSegment(
//                new PathLineSegment(
//                        new Vector2(-38.84, 45.98),
//                        new Vector2(-176.14, 77.15)
//                ), Rotation2.fromDegrees(10.0));
//        cargoSideNearToLoadingStationPathLeft.addSegment(
//                new PathLineSegment(
//                        new Vector2(-176.14, 77.15),
//                        new Vector2(-211.25, 85.13)
//                ), Rotation2.fromDegrees(0.0));
        cargoSideNearToLoadingStationPathLeft.subdivide(8);
        Path cargoSideNearToLoadingStationPathRight = cargoSideNearToLoadingStationPathLeft.mirror();
        cargoSideNearToLoadingStationTrajectoryLeft = new Trajectory(0.0, PICKUP_ENDING_VELOCITY, cargoSideNearToLoadingStationPathLeft, constraints);
        cargoSideNearToLoadingStationTrajectoryRight = new Trajectory(0.0, PICKUP_ENDING_VELOCITY, cargoSideNearToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Loading Station to Cargo Ship Side Mid">
        Path loadingStationToCargoSideMidPathLeft = new Path(Rotation2.fromDegrees(0.0));
        loadingStationToCargoSideMidPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(17.40, -4.61)
                ), Rotation2.fromDegrees(0.0));
        loadingStationToCargoSideMidPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(17.40, -4.61),
                        new Vector2(188.25, -49.9)
                ), Rotation2.fromDegrees(90.0));
        loadingStationToCargoSideMidPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(188.25, -49.9),
                        new Vector2(260.75, -69.13)
                ), Rotation2.fromDegrees(90.0));
        loadingStationToCargoSideMidPathLeft.subdivide(8);
        Path loadingStationToCargoSideMidPathRight = loadingStationToCargoSideMidPathLeft.mirror();
        loadingStationToCargoSideMidTrajectoryLeft = new Trajectory(loadingStationToCargoSideMidPathLeft, constraints);
        loadingStationToCargoSideMidTrajectoryRight = new Trajectory(loadingStationToCargoSideMidPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Cargo Ship Side Mid to Loading Station">
        Path cargoSideMidToLoadingStationPathLeft = new Path(Rotation2.fromDegrees(90.0));
        cargoSideMidToLoadingStationPathLeft.addSegment(new PathLineSegment(
                new Vector2(0.0, 0.0),
                new Vector2(-182.75, 57.13)
        ), Rotation2.fromDegrees(0.0));
        cargoSideMidToLoadingStationPathLeft.subdivide(8);
        Path cargoSideMidToLoadingStationPathRight = cargoSideMidToLoadingStationPathLeft.mirror();
        cargoSideMidToLoadingStationTrajectoryLeft = new Trajectory(cargoSideMidToLoadingStationPathLeft, constraints);
        cargoSideMidToLoadingStationTrajectoryRight = new Trajectory(cargoSideMidToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Loading Station to Cargo Ship Side Far">
        Path loadingStationToCargoSideFarPathLeft = new Path(Rotation2.ZERO);
        loadingStationToCargoSideFarPathLeft.addSegment(new PathLineSegment(
                new Vector2(0.0, 0.0),
                new Vector2(17.49, -4.25)
        ), Rotation2.ZERO);
        loadingStationToCargoSideFarPathLeft.addSegment(new PathLineSegment(
                new Vector2(17.49, -4.25),
                new Vector2(211.37, -51.4)
        ), Rotation2.fromDegrees(90.0));
        loadingStationToCargoSideFarPathLeft.addSegment(new PathLineSegment(
                new Vector2(211.37, -51.4),
                new Vector2(284.25, -69.13)
        ));
        loadingStationToCargoSideFarPathLeft.subdivide(8);
        Path loadingStationToCargoSideFarPathRight = loadingStationToCargoSideFarPathLeft.mirror();
        loadingStationToCargoSideFarTrajectoryLeft = new Trajectory(loadingStationToCargoSideFarPathLeft, constraints);
        loadingStationToCargoSideFarTrajectoryRight = new Trajectory(loadingStationToCargoSideFarPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Loading Station to Rocket Near">
        Path loadingStationToRocketNearPathLeft = new Path(Rotation2.ZERO);
        loadingStationToRocketNearPathLeft.addSegment(new PathLineSegment(
                new Vector2(0.0, 0.0),
                new Vector2(17.83, -2.48)
        ), Rotation2.ZERO);
        loadingStationToRocketNearPathLeft.addSegment(new PathLineSegment(
                new Vector2(17.83, -2.48),
                new Vector2(99.11, -13.77)
        ), Rotation2.fromDegrees(-151.25));
        loadingStationToRocketNearPathLeft.addSegment(new PathLineSegment(
                new Vector2(99.11, -13.77),
                new Vector2(146.65, -20.37)
        ));
        loadingStationToRocketNearPathLeft.subdivide(8);
        Path loadingStationToRocketNearPathRight = loadingStationToRocketNearPathLeft.mirror();
        loadingStationToRocketNearTrajectoryLeft = new Trajectory(loadingStationToRocketNearPathLeft, constraints);
        loadingStationToRocketNearTrajectoryRight = new Trajectory(loadingStationToRocketNearPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Rocket Near to Loading Station>
        Path rocketNearToLoadingStationPathLeft = new Path(Rotation2.fromDegrees(-151.25));
        rocketNearToLoadingStationPathLeft.addSegment(new PathLineSegment(
                new Vector2(0.0, 0.0),
                new Vector2(-86.96, -14.06)
        ), Rotation2.ZERO);
        Path rocketNearToLoadingStationPathRight = rocketNearToLoadingStationPathLeft.mirror();
        rocketNearToLoadingStationTrajectoryLeft = new Trajectory(rocketNearToLoadingStationPathLeft, constraints);
        rocketNearToLoadingStationTrajectoryRight = new Trajectory(rocketNearToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Loading Station to Rocket Far">
        Path loadingStationToRocketFarPathLeft = new Path(Rotation2.ZERO);
        loadingStationToRocketFarPathLeft.addSegment(new PathLineSegment(
                new Vector2(0.0, 0.0),
                new Vector2(17.72, -3.16)
        ), Rotation2.ZERO);
        loadingStationToRocketFarPathLeft.addSegment(new PathLineSegment(
                new Vector2(17.72, -3.16),
                new Vector2(191.44, -34.17)
        ), Rotation2.fromDegrees(-28.75));
        loadingStationToRocketFarPathLeft.addSegment(new PathArcSegment(
                new Vector2(191.44, -34.17),
                new Vector2(255.04, -5.94),
                new Vector2(202.36, 27.0)
        ));
        loadingStationToRocketFarPathLeft.subdivide(8);
        Path loadingStationToRocketFarPathRight = loadingStationToRocketFarPathLeft.mirror();
        loadingStationToRocketFarTrajectoryLeft = new Trajectory(loadingStationToRocketFarPathLeft, constraints);
        loadingStationToRocketFarTrajectoryRight = new Trajectory(loadingStationToRocketFarPathRight, constraints);
        // </editor-fold>
    }

    public Trajectory getHab1ToCargoSideNearTrajectory(Side side) {
        if (side == Side.LEFT) {
            return hab1ToCargoSideNearTrajectoryLeft;
        } else {
            return hab1ToCargoSideNearTrajectoryRight;
        }
    }

    public Trajectory getHab2ToCargoSideNearTrajectory(Side side) {
        if (side == Side.LEFT) {
            return hab2ToCargoSideNearTrajectoryLeft;
        } else {
            return hab2ToCargoSideNearTrajectoryRight;
        }
    }

    public Trajectory getCargoSideNearToLoadingStationTrajectory(Side side) {
        if (side == Side.LEFT) {
            return cargoSideNearToLoadingStationTrajectoryLeft;
        } else {
            return cargoSideNearToLoadingStationTrajectoryRight;
        }
    }

    public Trajectory getLoadingStationToCargoSideMidTrajectory(Side side) {
        if (side == Side.LEFT) {
            return loadingStationToCargoSideMidTrajectoryLeft;
        } else {
            return loadingStationToCargoSideMidTrajectoryRight;
        }
    }

    public Trajectory getCargoSideMidToLoadingStationTrajectory(Side side) {
        if (side == Side.LEFT) {
            return cargoSideMidToLoadingStationTrajectoryLeft;
        } else {
            return cargoSideMidToLoadingStationTrajectoryRight;
        }
    }

    public Trajectory getLoadingStationToCargoSideFarTrajectory(Side side) {
        if (side == Side.LEFT) {
            return loadingStationToCargoSideFarTrajectoryLeft;
        } else {
            return loadingStationToCargoSideFarTrajectoryRight;
        }
    }

    public Trajectory getLoadingStationToRocketNearTrajectory(Side side) {
        if (side == Side.LEFT) {
            return loadingStationToRocketNearTrajectoryLeft;
        } else {
            return loadingStationToRocketNearTrajectoryRight;
        }
    }

    public Trajectory getRocketNearToLoadingStationTrajectory(Side side) {
        if (side == Side.LEFT) {
            return rocketNearToLoadingStationTrajectoryLeft;
        } else {
            return rocketNearToLoadingStationTrajectoryRight;
        }
    }

    public Trajectory getLoadingStationToRocketFarTrajectory(Side side) {
        if (side == Side.LEFT) {
            return loadingStationToRocketFarTrajectoryLeft;
        } else {
            return loadingStationToRocketFarTrajectoryRight;
        }
    }
}
