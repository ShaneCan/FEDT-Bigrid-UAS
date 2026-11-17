package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import java.util.*;
import org.junit.jupiter.api.Test;

class BigraphGateTest {
  @Test
  void currentStateRequiresTheActualSingletonFlyingOccupant() throws Exception {
    var grid = GridGraph.rectangular(2, 2, 1, Set.of());
    var gate = new BigraphGate(grid, 3);
    var state = new BigraphGate.Snapshot(3, List.of(new BigraphGate.Occupant(0, 0, true)));
    assertTrue(gate.checkState(state, 0, 0).allowed());
    assertFalse(gate.checkState(state, 0, 1).allowed());
    assertFalse(gate.checkState(state, 1, 0).allowed());
    assertFalse(
        gate.checkState(
                new BigraphGate.Snapshot(
                    4,
                    List.of(
                        new BigraphGate.Occupant(0, 0, true),
                        new BigraphGate.Occupant(1, 0, true))),
                0,
                0)
            .allowed());
  }

  @Test
  void exactDirectedEdgeAndOccupancyMatchLocalAndFull() throws Exception {
    var grid = GridGraph.rectangular(3, 3, 1, Set.of());
    var gate = new BigraphGate(grid, 12);
    var snapshot =
        new BigraphGate.Snapshot(
            7,
            List.of(new BigraphGate.Occupant(10, 0, true), new BigraphGate.Occupant(2, 3, true)));
    for (int target : List.of(1, 3, 4)) {
      boolean local = gate.check(snapshot, 10, 0, target, false).allowed();
      boolean full = gate.check(snapshot, 10, 0, target, true).allowed();
      assertEquals(full, local);
      assertEquals(target == 1, local, "Destination and diagonal support occupancy must be exact");
    }
    assertFalse(
        gate.check(snapshot, 10, 1, 2, false).allowed(),
        "A different free neighbor cannot authorize this edge");
    assertFalse(
        gate.check(snapshot, 2, 0, 1, false).allowed(), "Rule is bound to the drone identity");
  }

  @Test
  void duplicateOccupancyAndUnavailableStateAreNeverCleanedAway() throws Exception {
    var grid = GridGraph.rectangular(2, 2, 1, Set.of());
    var gate = new BigraphGate(grid, 3);
    var duplicate =
        new BigraphGate.Snapshot(
            1,
            List.of(
                new BigraphGate.Occupant(0, 0, true),
                new BigraphGate.Occupant(1, 1, true),
                new BigraphGate.Occupant(2, 1, true)));
    assertFalse(gate.check(duplicate, 0, 0, 1, false).allowed());
    assertFalse(
        gate.check(
                new BigraphGate.Snapshot(2, List.of(new BigraphGate.Occupant(0, 0, false))),
                0,
                0,
                1,
                false)
            .allowed());
  }
}
