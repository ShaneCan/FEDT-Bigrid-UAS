package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import java.util.*;
import org.eclipse.emf.ecore.*;
import org.junit.jupiter.api.Test;

class SnapshotStoreTest {
  @Test
  void storageUsesAnIsolatedSchemaWithoutMutatingTheMatcherModel() throws Exception {
    var grid = GridGraph.rectangular(2, 2, 1, Set.of());
    var gate = new BigraphGate(grid, 2);
    var model =
        gate.world(new BigraphGate.Snapshot(1, List.of(new BigraphGate.Occupant(0, 0, true))));
    String original = model.getMetaModel().getNsURI();
    var schema = SnapshotStore.isolatedSchema(model);
    var copy = SnapshotStore.storageCopy(model, schema);
    assertEquals(original, model.getMetaModel().getNsURI());
    assertNotEquals(original, schema.getNsURI());
    assertEquals(schema, copy.eClass().getEPackage());
    assertEquals(count(model.getInstanceModel()), count(copy));
    assertEquals(schema.getNsURI(), SnapshotStore.isolatedSchema(model).getNsURI());
    assertNotSame(model.getInstanceModel(), copy);
  }

  private int count(EObject object) {
    int n = 0;
    var it = object.eAllContents();
    while (it.hasNext()) {
      it.next();
      n++;
    }
    return n;
  }
}
