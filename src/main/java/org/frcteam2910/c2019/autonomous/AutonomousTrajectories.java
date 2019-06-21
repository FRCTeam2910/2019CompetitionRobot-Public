package org.frcteam2910.c2019.autonomous;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.Side;

public class AutonomousTrajectories {
    private final Trajectory hab1ToCargoSideNearTrajectoryLeft;
    private final Trajectory hab1ToCargoSideNearTrajectoryRight;

    private final Trajectory hab2ToCargoSideNearTrajectoryLeft;
    private final Trajectory hab2ToCargoSideNearTrajectoryRight;

    private final Trajectory cargoSideNearToLoadingStationTrajectoryLeft;
    private final Trajectory cargoSideNearToLoadingStationTrajectoryRight;

    private final Trajectory cargoSideMidToLoadingStationTrajectoryLeft;
    private final Trajectory cargoSideMidToLoadingStationTrajectoryRight;

    private final Trajectory loadingStationToCargoSideMidTrajectoryLeft;
    private final Trajectory loadingStationToCargoSideMidTrajectoryRight;

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
                new Vector2(265.5, 17.13)
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
                        new Vector2(-3.11, 11.59)
                ), Rotation2.fromDegrees(90.0));
        cargoSideNearToLoadingStationPathLeft.addSegment(
                new PathArcSegment(
                        new Vector2(-3.11, 11.59),
                        new Vector2(-38.84, 45.98),
                        new Vector2(-49.47, -0.83)
                ));
        cargoSideNearToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(-38.84, 45.98),
                        new Vector2(-176.14, 77.15)
                ), Rotation2.fromDegrees(10.0));
        cargoSideNearToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(-176.14, 77.15),
                        new Vector2(-211.25, 85.13)
                ), Rotation2.fromDegrees(0.0));
        cargoSideNearToLoadingStationPathLeft.subdivide(8);
        Path cargoSideNearToLoadingStationPathRight = cargoSideNearToLoadingStationPathLeft.mirror();
        cargoSideNearToLoadingStationTrajectoryLeft = new Trajectory(cargoSideNearToLoadingStationPathLeft, constraints);
        cargoSideNearToLoadingStationTrajectoryRight = new Trajectory(cargoSideNearToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Cargo Ship Side Mid to Loading Station">
        Path cargoSideMidToLoadingStationPathLeft = new Path(Rotation2.fromDegrees(90.0));
        cargoSideMidToLoadingStationPathLeft.addSegment(new PathLineSegment(
                new Vector2(0.0, 0.0),
                new Vector2(-2.07, 7.73)
        ), Rotation2.fromDegrees(90.0));
        cargoSideMidToLoadingStationPathLeft.addSegment(new PathArcSegment(
                new Vector2(-2.07, 7.73),
                new Vector2(-36.29, 41.74),
                new Vector2(-48.44, -4.70)
        ));
        cargoSideMidToLoadingStationPathLeft.addSegment(new PathLineSegment(
                new Vector2(-36.29, 41.74),
                new Vector2(-136.0, 67.82)
        ), Rotation2.fromDegrees(0.0));
        cargoSideMidToLoadingStationPathLeft.addSegment(new PathLineSegment(
                new Vector2(-136.0, 67.82),
                new Vector2(-232.75, 93.13)
        ), Rotation2.fromDegrees(0.0));
        cargoSideMidToLoadingStationPathLeft.subdivide(8);
        Path cargoSideMidToLoadingStationPathRight = cargoSideMidToLoadingStationPathLeft.mirror();
        cargoSideMidToLoadingStationTrajectoryLeft = new Trajectory(cargoSideMidToLoadingStationPathLeft, constraints);
        cargoSideMidToLoadingStationTrajectoryRight = new Trajectory(cargoSideMidToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Loading Station to Cargo Ship Side Mid">
        Path loadingStationToCargoSideMidPathLeft = new Path(Rotation2.fromDegrees(0.0));
        loadingStationToCargoSideMidPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(17.28, -5.05)
                ), Rotation2.fromDegrees(0.0));
        loadingStationToCargoSideMidPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(17.28, -5.05),
                        new Vector2(191.77, -56.08)
                ), Rotation2.fromDegrees(90.0));
        loadingStationToCargoSideMidPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(191.77, -56.08),
                        new Vector2(263.75, -77.13)
                ), Rotation2.fromDegrees(90.0));
        loadingStationToCargoSideMidPathLeft.subdivide(8);
        Path loadingStationToCargoSideMidPathRight = loadingStationToCargoSideMidPathLeft.mirror();
        loadingStationToCargoSideMidTrajectoryLeft = new Trajectory(loadingStationToCargoSideMidPathLeft, constraints);
        loadingStationToCargoSideMidTrajectoryRight = new Trajectory(loadingStationToCargoSideMidPathRight, constraints);
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

    public Trajectory getCargoSideMidToLoadingStationTrajectory(Side side) {
        if (side == Side.LEFT) {
            return cargoSideMidToLoadingStationTrajectoryLeft;
        } else {
            return cargoSideMidToLoadingStationTrajectoryRight;
        }
    }

    public Trajectory getLoadingStationToCargoSideMid(Side side) {
        if (side == Side.LEFT) {
            return loadingStationToCargoSideMidTrajectoryLeft;
        } else {
            return loadingStationToCargoSideMidTrajectoryRight;
        }
    }
}
