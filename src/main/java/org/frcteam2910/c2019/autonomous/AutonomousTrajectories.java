package org.frcteam2910.c2019.autonomous;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.Side;

public class AutonomousTrajectories {
    private static final double LOADING_STATION_ENDING_VELOCITY = 7.5 * 12.0;
    private static final double PLACEMENT_ENDING_VELOCITY = 3.0 * 12.0;

    private static final int SUBDIVIDE_ITERATIONS = 8;

    private static final Rotation2 LOADING_STATION_ROTATION = Rotation2.ZERO;
    private static final Rotation2 CARGO_SHIP_SIDE_HATCH_ROTATION = Rotation2.fromDegrees(90.0);
    private static final Rotation2 CARGO_SHIP_SIDE_CARGO_ROTATION = Rotation2.fromDegrees(-90.0);
    private static final Rotation2 ROCKET_FAR_ROTATION = Rotation2.fromDegrees(-28.75);
    private static final Rotation2 ROCKET_NEAR_ROTATION = Rotation2.fromDegrees(-151.25);
    private static final Rotation2 ROCKET_CARGO_ROTATION = Rotation2.fromDegrees(90.0);
    private static final Rotation2 DEPOT_ROTATION = Rotation2.fromDegrees(225.0);

    private final Trajectory hab1ToCargoSideNearTrajectoryLeft;
    private final Trajectory hab1ToCargoSideNearTrajectoryRight;

    private final Trajectory hab1ToRocketFarTrajectoryLeft;
    private final Trajectory hab1ToRocketFarTrajectoryRight;

    private final Trajectory hab2ToCargoSideNearTrajectoryLeft;
    private final Trajectory hab2ToCargoSideNearTrajectoryRight;

    private final Trajectory hab2ToRocketFarTrajectoryLeft;
    private final Trajectory hab2ToRocketFarTrajectoryRight;

    private final Trajectory cargoSideNearToLoadingStationTrajectoryLeft;
    private final Trajectory cargoSideNearToLoadingStationTrajectoryRight;

    private final Trajectory loadingStationToCargoSideMidTrajectoryLeft;
    private final Trajectory loadingStationToCargoSideMidTrajectoryRight;

    private final Trajectory cargoSideMidToLoadingStationTrajectoryLeft;
    private final Trajectory cargoSideMidToLoadingStationTrajectoryRight;

    private final Trajectory loadingStationToRocketNearTrajectoryLeft;
    private final Trajectory loadingStationToRocketNearTrajectoryRight;

    private final Trajectory rocketNearToLoadingStationTrajectoryLeft;
    private final Trajectory rocketNearToLoadingStationTrajectoryRight;

    private final Trajectory loadingStationToRocketFarTrajectoryLeft;
    private final Trajectory loadingStationToRocketFarTrajectoryRight;

    private final Trajectory rocketFarToLoadingStationTrajectoryLeft;
    private final Trajectory rocketFarToLoadingStationTrajectoryRight;

    private final Trajectory rocketNearToDepotTrajectoryLeft;
    private final Trajectory rocketNearToDepotTrajectoryRight;

    private final Trajectory depotToRocketCargoTrajectoryLeft;
    private final Trajectory depotToRocketCargoTrajectoryRight;

    private final Trajectory rocketCargoToCargoSideNearTrajectoryLeft;
    private final Trajectory rocketCargoToCargoSideNearTrajectoryRight;

    public AutonomousTrajectories(ITrajectoryConstraint... constraints) {
        // <editor-fold desc="Hab to Cargo Ship Side Near">
        Path habToCargoSideNearPathLeft = new Path(CARGO_SHIP_SIDE_HATCH_ROTATION);
        habToCargoSideNearPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(207.0, 12.13)
                )
        );
        habToCargoSideNearPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path habToCargoSideNearPathRight = habToCargoSideNearPathLeft.mirror();
        hab1ToCargoSideNearTrajectoryLeft = new Trajectory(0.0, PLACEMENT_ENDING_VELOCITY, habToCargoSideNearPathLeft, constraints);
        hab1ToCargoSideNearTrajectoryRight = new Trajectory(0.0, PLACEMENT_ENDING_VELOCITY, habToCargoSideNearPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Hab to Rocket Far">
        Path habToRocketFarPathLeft = new Path(LOADING_STATION_ROTATION);
        habToRocketFarPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(188.65, 39.78)
                )
        );
        habToRocketFarPathLeft.addSegment(
                new PathArcSegment(
                        new Vector2(188.65, 39.78),
                        new Vector2(207.93, 84.99),
                        new Vector2(182.46, 69.13)
                ),
                ROCKET_FAR_ROTATION
        );
        habToRocketFarPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path habToRocketFarPathRight = habToRocketFarPathLeft.mirror();
        hab1ToRocketFarTrajectoryLeft = new Trajectory(habToRocketFarPathLeft, constraints);
        hab1ToRocketFarTrajectoryRight = new Trajectory(habToRocketFarPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Hab 2 to Cargo Ship Side Near">
        Path hab2ToCargoSideNearPathLeft = new Path(CARGO_SHIP_SIDE_HATCH_ROTATION);
        hab2ToCargoSideNearPathLeft.addSegment(new PathLineSegment(
                new Vector2(0.0, 0.0),
                new Vector2(270.25, 36.88)
        ));
        hab2ToCargoSideNearPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path hab2ToCargoSideNearPathRight = hab2ToCargoSideNearPathLeft.mirror();
        hab2ToCargoSideNearTrajectoryLeft = new Trajectory(hab2ToCargoSideNearPathLeft, constraints);
        hab2ToCargoSideNearTrajectoryRight = new Trajectory(hab2ToCargoSideNearPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Hab 2 to Rocket Far">
        Path hab2ToRocketFarPathLeft = new Path(LOADING_STATION_ROTATION);
        hab2ToRocketFarPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(270.36, 61.48)
                )
        );
        hab2ToRocketFarPathLeft.addSegment(
                new PathArcSegment(
                        new Vector2(270.36, 61.48),
                        new Vector2(288.93, 106.99),
                        new Vector2(263.71, 90.73)
                ),
                ROCKET_FAR_ROTATION
        );
        hab2ToRocketFarPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path hab2ToRocketFarPathRight = hab2ToRocketFarPathLeft.mirror();
        hab2ToRocketFarTrajectoryLeft = new Trajectory(0.0, PLACEMENT_ENDING_VELOCITY, hab2ToRocketFarPathLeft, constraints);
        hab2ToRocketFarTrajectoryRight = new Trajectory(0.0, PLACEMENT_ENDING_VELOCITY, hab2ToRocketFarPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Cargo Ship Side Near to Loading Station">
        Path cargoSideNearToLoadingStationPathLeft = new Path(CARGO_SHIP_SIDE_HATCH_ROTATION);
        cargoSideNearToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(-3.64, 10.0)
                ),
                CARGO_SHIP_SIDE_HATCH_ROTATION
        );
        cargoSideNearToLoadingStationPathLeft.addSegment(
                new PathArcSegment(
                        new Vector2(-3.64, 10.0),
                        new Vector2(-38.15, 41.32),
                        new Vector2(-50.62, -7.1)
                )
        );
        cargoSideNearToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(-38.15, 41.32),
                        new Vector2(-161.25, 73.03)
                ), LOADING_STATION_ROTATION
        );
        cargoSideNearToLoadingStationPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path cargoSideNearToLoadingStationPathRight = cargoSideNearToLoadingStationPathLeft.mirror();
        cargoSideNearToLoadingStationTrajectoryLeft = new Trajectory(0.0, LOADING_STATION_ENDING_VELOCITY, cargoSideNearToLoadingStationPathLeft, constraints);
        cargoSideNearToLoadingStationTrajectoryRight = new Trajectory(0.0, LOADING_STATION_ENDING_VELOCITY, cargoSideNearToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Loading Station to Cargo Ship Side Mid">
        Path loadingStationToCargoSideMidPathLeft = new Path(LOADING_STATION_ROTATION);
        loadingStationToCargoSideMidPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(209.87, -50.62)
                ),
                CARGO_SHIP_SIDE_HATCH_ROTATION
        );
        loadingStationToCargoSideMidPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(221.77, -51.03),
                        new Vector2(260.75, -60.0)
                )
        );
        loadingStationToCargoSideMidPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path loadingStationToCargoSideMidPathRight = loadingStationToCargoSideMidPathLeft.mirror();
        loadingStationToCargoSideMidTrajectoryLeft = new Trajectory(loadingStationToCargoSideMidPathLeft, constraints);
        loadingStationToCargoSideMidTrajectoryRight = new Trajectory(loadingStationToCargoSideMidPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Cargo Ship Side Mid to Loading Station">
        Path cargoSideMidToLoadingStationPathLeft = new Path(CARGO_SHIP_SIDE_HATCH_ROTATION);
        cargoSideMidToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(-3.64, 10.0)
                ),
                CARGO_SHIP_SIDE_HATCH_ROTATION
        );
        cargoSideMidToLoadingStationPathLeft.addSegment(
                new PathArcSegment(
                        new Vector2(-3.64, 10.0),
                        new Vector2(-39.29, 41.6),
                        new Vector2(-50.62, -7.1)
                )
        );
        cargoSideMidToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(-39.29, 41.6),
                        new Vector2(-182.75, 74.98)
                ),
                LOADING_STATION_ROTATION
        );
        cargoSideMidToLoadingStationPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path cargoSideMidToLoadingStationPathRight = cargoSideMidToLoadingStationPathLeft.mirror();
        cargoSideMidToLoadingStationTrajectoryLeft = new Trajectory(0.0, LOADING_STATION_ENDING_VELOCITY, cargoSideMidToLoadingStationPathLeft, constraints);
        cargoSideMidToLoadingStationTrajectoryRight = new Trajectory(0.0, LOADING_STATION_ENDING_VELOCITY, cargoSideMidToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Loading Station to Rocket Near">
        Path loadingStationToRocketNearPathLeft = new Path(LOADING_STATION_ROTATION);
        loadingStationToRocketNearPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(102.33, -15.74)
                ),
                ROCKET_NEAR_ROTATION
        );
        loadingStationToRocketNearPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(102.33, -15.74),
                        new Vector2(130.0, -20.0)
                )
        );
        loadingStationToRocketNearPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path loadingStationToRocketNearPathRight = loadingStationToRocketNearPathLeft.mirror();
        loadingStationToRocketNearTrajectoryLeft = new Trajectory(loadingStationToRocketNearPathLeft, constraints);
        loadingStationToRocketNearTrajectoryRight = new Trajectory(loadingStationToRocketNearPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Rocket Near to Loading Station>
        Path rocketNearToLoadingStationPathLeft = new Path(ROCKET_NEAR_ROTATION);
        rocketNearToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(-98.35, 6.0)
                ),
                LOADING_STATION_ROTATION
        );
        rocketNearToLoadingStationPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path rocketNearToLoadingStationPathRight = rocketNearToLoadingStationPathLeft.mirror();
        rocketNearToLoadingStationTrajectoryLeft = new Trajectory(0.0, LOADING_STATION_ENDING_VELOCITY, rocketNearToLoadingStationPathLeft, constraints);
        rocketNearToLoadingStationTrajectoryRight = new Trajectory(0.0, LOADING_STATION_ENDING_VELOCITY, rocketNearToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Loading Station to Rocket Far">
        Path loadingStationToRocketFarPathLeft = new Path(LOADING_STATION_ROTATION);
        loadingStationToRocketFarPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(212.45, -45.26)
                )
        );
        loadingStationToRocketFarPathLeft.addSegment(
                new PathArcSegment(
                        new Vector2(212.45, -45.26),
                        new Vector2(248.68, -16.98),
                        new Vector2(218.7, -15.92)
                ),
                ROCKET_FAR_ROTATION
        );
        loadingStationToRocketFarPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path loadingStationToRocketFarPathRight = loadingStationToRocketFarPathLeft.mirror();
        loadingStationToRocketFarTrajectoryLeft = new Trajectory(loadingStationToRocketFarPathLeft, constraints);
        loadingStationToRocketFarTrajectoryRight = new Trajectory(loadingStationToRocketFarPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Rocket Far to Loading Station">
        Path rocketFarToLoadingStationPathLeft = new Path(ROCKET_FAR_ROTATION);
        rocketFarToLoadingStationPathLeft.addSegment(
                new PathArcSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(-38.81, -52.01),
                        new Vector2(-29.92, -18.16)
                ),
                ROCKET_FAR_ROTATION
        );
        rocketFarToLoadingStationPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(-38.81, -52.01),
                        new Vector2(-157.65, -20.8)
                ),
                LOADING_STATION_ROTATION
        );
        rocketFarToLoadingStationPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path rocketFarToLoadingStationPathRight = rocketFarToLoadingStationPathLeft.mirror();
        rocketFarToLoadingStationTrajectoryLeft = new Trajectory(0.0, LOADING_STATION_ENDING_VELOCITY, rocketFarToLoadingStationPathLeft, constraints);
        rocketFarToLoadingStationTrajectoryRight = new Trajectory(0.0, LOADING_STATION_ENDING_VELOCITY, rocketFarToLoadingStationPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Rocket Near to Depot">
        Path rocketNearToDepotPathLeft = new Path(ROCKET_NEAR_ROTATION);
        rocketNearToDepotPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(-133.28, -38.15)
                ),
                DEPOT_ROTATION
        );
        rocketNearToDepotPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path rocketNearToDepotPathRight = rocketNearToDepotPathLeft.mirror();
        rocketNearToDepotTrajectoryLeft = new Trajectory(rocketNearToDepotPathLeft, constraints);
        rocketNearToDepotTrajectoryRight = new Trajectory(rocketNearToDepotPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Depot to Rocket Cargo">
        Path depotToRocketCargoPathLeft = new Path(DEPOT_ROTATION);
        depotToRocketCargoPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(156.25, 8.86)
                ),
                ROCKET_CARGO_ROTATION
        );
        depotToRocketCargoPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path depotToRocketCargoPathRight = depotToRocketCargoPathLeft.mirror();
        depotToRocketCargoTrajectoryLeft = new Trajectory(depotToRocketCargoPathLeft, constraints);
        depotToRocketCargoTrajectoryRight = new Trajectory(depotToRocketCargoPathRight, constraints);
        // </editor-fold>

        // <editor-fold desc="Rocket Cargo to Cargo Side Near">
        Path rocketCargoToCargoSideNearPathLeft = new Path(ROCKET_CARGO_ROTATION);
        rocketCargoToCargoSideNearPathLeft.addSegment(
                new PathLineSegment(
                        new Vector2(0.0, 0.0),
                        new Vector2(33.25, -47.38)
                ),
                CARGO_SHIP_SIDE_CARGO_ROTATION
        );
        rocketCargoToCargoSideNearPathLeft.subdivide(SUBDIVIDE_ITERATIONS);
        Path rocketCargoToCargoSideNearPathRight = rocketCargoToCargoSideNearPathLeft.mirror();
        rocketCargoToCargoSideNearTrajectoryLeft = new Trajectory(rocketCargoToCargoSideNearPathLeft, constraints);
        rocketCargoToCargoSideNearTrajectoryRight = new Trajectory(rocketCargoToCargoSideNearPathRight, constraints);
        // </editor-fold>
    }

    public Trajectory getHab1ToCargoSideNearTrajectory(Side side) {
        if (side == Side.LEFT) {
            return hab1ToCargoSideNearTrajectoryLeft;
        } else {
            return hab1ToCargoSideNearTrajectoryRight;
        }
    }

    public Trajectory getHab1ToRocketFarTrajectory(Side side) {
        if (side == Side.LEFT) {
            return hab1ToRocketFarTrajectoryLeft;
        } else {
            return hab1ToRocketFarTrajectoryRight;
        }
    }

    public Trajectory getHab2ToCargoSideNearTrajectory(Side side) {
        if (side == Side.LEFT) {
            return hab2ToCargoSideNearTrajectoryLeft;
        } else {
            return hab2ToCargoSideNearTrajectoryRight;
        }
    }

    public Trajectory getHab2ToRocketFarTrajectory(Side side) {
        if (side == Side.LEFT) {
            return hab2ToRocketFarTrajectoryLeft;
        } else {
            return hab2ToRocketFarTrajectoryRight;
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

    public Trajectory getRocketFarToLoadingStationTrajectory(Side side) {
        if (side == Side.LEFT) {
            return rocketFarToLoadingStationTrajectoryLeft;
        } else {
            return rocketFarToLoadingStationTrajectoryRight;
        }
    }

    public Trajectory getRocketNearToDepotTrajectory(Side side) {
        if (side == Side.LEFT) {
            return rocketNearToDepotTrajectoryLeft;
        } else {
            return rocketNearToDepotTrajectoryRight;
        }
    }

    public Trajectory getDepotToRocketCargoTrajectory(Side side) {
        if (side == Side.LEFT) {
            return depotToRocketCargoTrajectoryLeft;
        } else {
            return depotToRocketCargoTrajectoryRight;
        }
    }

    public Trajectory getRocketCargoToCargoSideNearTrajectory(Side side) {
        if (side == Side.LEFT) {
            return rocketCargoToCargoSideNearTrajectoryLeft;
        } else {
            return rocketCargoToCargoSideNearTrajectoryRight;
        }
    }
}
