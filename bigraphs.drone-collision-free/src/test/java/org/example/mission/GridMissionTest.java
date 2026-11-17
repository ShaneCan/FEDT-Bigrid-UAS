package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import java.util.*;
import org.junit.jupiter.api.Test;

class GridMissionTest {

  @Test
  void adjacentHeldCentersCanReplanWhenEnvelopeCellSetsOverlap() {
    var template = GridGraph.rectangular(11, 7, 3, Set.of());
    var g =
        new GridGraph(
            11,
            7,
            3,
            new Vec3(-1.5, -2.5, 0),
            new Vec3(.5, .5, .6),
            Set.of(28, 105),
            template.links());
    var planner = new CooperativePlanner(g, .15, .3, .1, 96, 3000);
    var p0 = new Vec3(.3563947363, -.1208120657, .3);
    var p1 = new Vec3(.5, .3225862488, .3);
    assertNotEquals(g.cell(p0), g.cell(p1));
    assertFalse(Collections.disjoint(g.footprint(p0, .1), g.footprint(p1, .1)));
    var plan =
        assertDoesNotThrow(
            () ->
                planner.plan(
                    List.of(
                        new CooperativePlanner.Agent(
                            0, g.cell(p0), g.cell(new Vec3(0, -2.5, .3)), p0),
                        new CooperativePlanner.Agent(
                            1, g.cell(p1), g.cell(new Vec3(.5, -2, .3)), p1))));
    assertDoesNotThrow(() -> planner.certify(plan.trajectories()));
    var close0 = new Vec3(.5, .24, .3);
    var close1 = new Vec3(.5, .26, .3);
    assertThrows(
        RuntimeException.class,
        () ->
            planner.plan(
                List.of(
                    new CooperativePlanner.Agent(
                        0, g.cell(close0), g.cell(new Vec3(0, -2.5, .3)), close0),
                    new CooperativePlanner.Agent(
                        1, g.cell(close1), g.cell(new Vec3(.5, -2, .3)), close1))));
  }

  @Test
  void initialEnvelopeStillRespectsUnmodeledFixedReservations() {
    var g = GridGraph.rectangular(3, 4, 1, Set.of());
    var planner = new CooperativePlanner(g, .15, .3, .1, 30, 1000);
    var measured = new Vec3(.26, 0, .3);
    assertThrows(
        RuntimeException.class,
        () ->
            planner.plan(
                List.of(new CooperativePlanner.Agent(0, g.cell(measured), 9, measured)),
                t -> Set.of(0)));
  }

  @Test
  void defaultTwelveAircraftStillHaveACertifiedJointPlan() {
    var cfg = MissionConfig.from(new org.springframework.mock.env.MockEnvironment());
    var template = GridGraph.rectangular(cfg.rows(), cfg.cols(), cfg.layers(), cfg.obstacles());
    var g =
        new GridGraph(
            cfg.rows(),
            cfg.cols(),
            cfg.layers(),
            cfg.origin(),
            cfg.step(),
            cfg.obstacles(),
            template.links());
    double[][] initial = {
      {1, 1.5},
      {-1, 1.5},
      {-1.5, 1},
      {-1.5, -1},
      {-1, -1.5},
      {1, -1.5},
      {1.5, -1},
      {1.5, 1},
      {1, 1},
      {-1, 1},
      {-1, -1},
      {1, -1}
    };
    var agents = new ArrayList<CooperativePlanner.Agent>();
    for (int id = 0; id < initial.length; id++) {
      var p = new Vec3(initial[id][0], initial[id][1], .3);
      agents.add(new CooperativePlanner.Agent(id, g.cell(p), cfg.goals(g).get(id), p));
    }
    var planner =
        new CooperativePlanner(
            g, cfg.speed(), cfg.acceleration(), cfg.margin(), cfg.horizon(), cfg.nodeLimit());
    var plan = planner.plan(agents);
    assertEquals(12, plan.trajectories().size());
    assertDoesNotThrow(() -> planner.certify(plan.trajectories()));
  }

  @Test
  void changedGridConfigurationChangesGeometryGoalsAndTrajectory() {
    var cfg =
        MissionConfig.from(
            new org.springframework.mock.env.MockEnvironment()
                .withProperty("bigrid.service.rows", "3")
                .withProperty("bigrid.service.cols", "4")
                .withProperty("bigrid.service.layers", "2")
                .withProperty("bigrid.service.origin.x", "0")
                .withProperty("bigrid.service.origin.y", "0")
                .withProperty("bigrid.service.step.x", "0.4")
                .withProperty("bigrid.service.step.y", "0.3")
                .withProperty("bigrid.service.layer.height", "0.5")
                .withProperty("drone.count", "1")
                .withProperty("obstacle.grids", "")
                .withProperty("drone.targets", "0.8,0.3,0.75"));
    var template = GridGraph.rectangular(cfg.rows(), cfg.cols(), cfg.layers(), Set.of());
    var g =
        new GridGraph(
            cfg.rows(),
            cfg.cols(),
            cfg.layers(),
            cfg.origin(),
            cfg.step(),
            cfg.obstacles(),
            template.links());
    int goal = cfg.goals(g).get(0);
    assertEquals(19, goal);
    assertEquals(new Vec3(.8, .3, .75), g.center(goal));
    var planner =
        new CooperativePlanner(
            g, cfg.speed(), cfg.acceleration(), cfg.margin(), cfg.horizon(), cfg.nodeLimit());
    var curve =
        planner
            .plan(List.of(new CooperativePlanner.Agent(0, 0, goal, g.center(0))))
            .trajectories()
            .get(0);
    assertTrue(curve.at(curve.duration()).distance(g.center(goal)) < 1e-9);
    for (int i = 1; i < curve.path.size(); i++)
      assertTrue(g.legal(curve.path.get(i - 1), curve.path.get(i)));
    assertEquals(.1, cfg.margin(), "The physical envelope does not scale with the cells");
  }

  @Test
  void geometryRejectsOutsideAndUsesPhysicalLayerCenters() {
    var g = GridGraph.rectangular(3, 4, 3, Set.of());
    assertEquals(new Vec3(0, 0, .3), g.center(0));
    assertEquals(0, g.cell(new Vec3(-.249, 0, .01)));
    assertThrows(IllegalArgumentException.class, () -> g.cell(new Vec3(-.251, 0, .3)));
    assertThrows(IllegalArgumentException.class, () -> g.cell(new Vec3(0, 0, -.001)));
    assertEquals(12, g.cell(new Vec3(0, 0, .6)));
    assertFalse(g.legal(0, 13));
    assertEquals("ForwardLeftRoute", g.route(0, 4));
  }

  @Test
  void diagonalCannotCutBlockedCornerOrInventOneWayEdge() {
    var g = GridGraph.rectangular(3, 3, 1, Set.of(1));
    assertFalse(g.legal(0, 4));
    var directed =
        new GridGraph(
            2,
            2,
            1,
            Vec3.ZERO,
            new Vec3(.5, .5, .6),
            Set.of(),
            List.of(new GridGraph.Edge(0, 2, "ForwardRoute")));
    assertTrue(directed.legal(0, 2));
    assertFalse(directed.legal(2, 0));
    assertFalse(directed.legal(0, 1));
  }

  @Test
  void septicIsContinuousAndDoesNotStopAtOrdinaryCellBoundary() {
    var g = GridGraph.rectangular(3, 4, 1, Set.of());
    double slot = 12;
    var c = new PolynomialTrajectory(g, List.of(0, 3, 6, 9), g.center(0), slot, .15, .3, .1);
    assertEquals(3, c.crossings(g).size());
    for (int i = 0; i < c.pieces.size() - 1; i++) {
      var a = c.pieces.get(i).control();
      var b = c.pieces.get(i + 1).control();
      assertTrue(a[7].distance(b[0]) < 1e-10);
      for (int derivative = 1; derivative <= 3; derivative++) {
        a = difference(a);
        b = difference(b);
        assertTrue(a[a.length - 1].distance(b[0]) < 1e-9, "C" + derivative);
      }
    }
    var p = c.pieces.get(0).control();
    assertTrue(p[7].sub(p[6]).norm() > 1e-5);
    for (var piece : c.pieces) assertTrue(PolynomialTrajectory.certifyCorridor(g, piece, .1));
  }

  private Vec3[] difference(Vec3[] p) {
    Vec3[] d = new Vec3[p.length - 1];
    for (int i = 0; i < d.length; i++) d[i] = p[i + 1].sub(p[i]).mul(p.length - 1);
    return d;
  }

  @Test
  void aLaterDiagonalTurnDoesNotStopEarlierStraightSegments() {
    var g = GridGraph.rectangular(4, 4, 1, Set.of());
    var curve = new PolynomialTrajectory(g, List.of(0, 4, 8, 9, 14), g.center(0), 12, .15, .3, .1);
    var control = curve.pieces.getFirst().control();
    assertTrue(control[7].sub(control[6]).norm() > 1e-5);
  }

  @Test
  void restartNearACellFaceIncludesItsInitialSafetyFootprint() {
    var g = GridGraph.rectangular(3, 4, 1, Set.of());
    var p = new CooperativePlanner(g, .15, .3, .1, 30, 1000);
    Vec3 measured = new Vec3(.26, 0, .3);
    int current = g.cell(measured);
    var curve =
        p.plan(List.of(new CooperativePlanner.Agent(0, current, 9, measured)))
            .trajectories()
            .get(0);
    assertEquals(measured, curve.at(0));
    assertEquals(current, curve.path.get(1));
    assertTrue(curve.initialCells.contains(0));
  }

  @Test
  void continuousEnvelopeRejectsOvershootMissedByEndpoints() {
    var g = GridGraph.rectangular(3, 3, 1, Set.of());
    Vec3[] control = new Vec3[8];
    Arrays.fill(control, g.center(0));
    control[7] = g.center(3);
    control[3] = new Vec3(.2, -2, .3);
    var piece = new PolynomialTrajectory.Piece(12, control, 0, 3);
    assertFalse(PolynomialTrajectory.certifyCorridor(g, piece, .1));
  }

  @Test
  void unionCoverageAllowsSharedFacesButNotHoles() {
    var g = GridGraph.rectangular(2, 2, 1, Set.of());
    var envelope = new GridGraph.Box(new Vec3(-.1, -.1, .2), new Vec3(.6, .1, .4));
    assertTrue(PolynomialTrajectory.covered(envelope, List.of(g.box(0), g.box(2))));
    assertFalse(PolynomialTrajectory.covered(envelope, List.of(g.box(0), g.box(3))));
  }

  @Test
  void cbsProducesWholeConcurrentConflictFreeMissions() {
    var g = GridGraph.rectangular(4, 4, 1, Set.of());
    var p = new CooperativePlanner(g, .15, .3, .1, 30, 2000);
    var plan =
        p.plan(
            List.of(
                new CooperativePlanner.Agent(10, 0, 15, g.center(0)),
                new CooperativePlanner.Agent(2, 3, 12, g.center(3))));
    assertEquals(Set.of(2, 10), plan.trajectories().keySet());
    var a = plan.trajectories().get(2);
    var b = plan.trajectories().get(10);
    for (double t = 0; t < Math.max(a.duration(), b.duration()); t += .05)
      assertTrue(a.at(t).distance(b.at(t)) >= .2 - 1e-8);
    assertEquals(g.center(12), a.at(a.duration()));
    assertEquals(g.center(15), b.at(b.duration()));
  }

  @Test
  void invalidPermanentGoalsAndUnsolvableSwapFailBeforeDispatch() {
    var g = GridGraph.rectangular(1, 2, 1, Set.of());
    var p = new CooperativePlanner(g, .15, .3, .1, 8, 100);
    assertThrows(
        IllegalStateException.class,
        () ->
            p.plan(
                List.of(
                    new CooperativePlanner.Agent(0, 0, 1, g.center(0)),
                    new CooperativePlanner.Agent(1, 1, 1, g.center(1)))));
    assertThrows(
        IllegalStateException.class,
        () ->
            p.plan(
                List.of(
                    new CooperativePlanner.Agent(0, 0, 1, g.center(0)),
                    new CooperativePlanner.Agent(1, 1, 0, g.center(1)))));
  }

  @Test
  void defaultTwelveAircraftMissionHasACompleteJointPlan() {
    var cfg = MissionConfig.from(new org.springframework.mock.env.MockEnvironment());
    var template = GridGraph.rectangular(7, 7, 3, Set.of());
    var g = new GridGraph(7, 7, 3, cfg.origin(), cfg.step(), cfg.obstacles(), template.links());
    double[][] xy = {
      {1, 1.5},
      {-1, 1.5},
      {-1.5, 1},
      {-1.5, -1},
      {-1, -1.5},
      {1, -1.5},
      {1.5, -1},
      {1.5, 1},
      {1, 1},
      {-1, 1},
      {-1, -1},
      {1, -1}
    };
    List<CooperativePlanner.Agent> agents = new ArrayList<>();
    var goals = cfg.goals(g);
    for (int id = 0; id < 12; id++) {
      var pos = new Vec3(xy[id][0], xy[id][1], .3);
      agents.add(new CooperativePlanner.Agent(id, g.cell(pos), goals.get(id), pos));
    }
    var p = new CooperativePlanner(g, .15, .3, .1, 96, 3000);
    var plan = p.plan(agents);
    assertEquals(12, plan.trajectories().size());
  }

  @Test
  void replanStartsAtMeasuredHoldAndRespectsFixedReservations() {
    var g = GridGraph.rectangular(4, 4, 1, Set.of());
    var p = new CooperativePlanner(g, .15, .3, .1, 30, 1000);
    Vec3 actual = g.center(0).add(new Vec3(.08, .02, 0));
    var plan = p.plan(List.of(new CooperativePlanner.Agent(0, 0, 15, actual)), slot -> Set.of(5));
    var curve = plan.trajectories().get(0);
    assertEquals(actual, curve.at(0));
    for (int i = 0; i < curve.path.size() - 1; i++)
      assertFalse(g.corridor(curve.path.get(i), curve.path.get(i + 1)).contains(5));
  }
}
