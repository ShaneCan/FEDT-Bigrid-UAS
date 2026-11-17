package org.example.mission;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicReference;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.spring.data.cdo.*;
import org.eclipse.emf.ecore.*;
import org.eclipse.emf.ecore.util.EcoreUtil;

/** A single pending snapshot bounds persistence work; CDO never owns matcher objects. */
public final class SnapshotStore implements AutoCloseable {
  private final ScheduledExecutorService worker = Executors.newSingleThreadScheduledExecutor();
  private final AtomicReference<BigraphGate.Snapshot> pending = new AtomicReference<>();
  private final BigraphGate gate;
  private CdoTemplate template;
  private SimpleCdoDbFactory factory;
  private EPackage schema;
  private final String connection;
  public volatile String lastError;

  public SnapshotStore(BigraphGate gate, String connection) {
    this.gate = gate;
    this.connection = connection;
    worker.scheduleWithFixedDelay(this::flush, 0, 500, TimeUnit.MILLISECONDS);
  }

  public void submit(BigraphGate.Snapshot snapshot) {
    pending.set(snapshot);
  }

  private void flush() {
    var snapshot = pending.getAndSet(null);
    if (snapshot == null) return;
    try {
      if (template == null) {
        factory = new SimpleCdoDbFactory(new CdoServerConnectionString(connection));
        template = new CdoTemplate(factory);
      }
      var model = gate.world(snapshot);
      if (schema == null) {
        schema = isolatedSchema(model);
        template.getCDOPackageRegistry().put(schema.getNsURI(), schema);
      }
      var copy = storageCopy(model, schema);
      template.removeAll("/cooperative/live");
      template.insert(copy, "/cooperative/live");
      lastError = null;
    } catch (Exception e) {
      String error = e.toString();
      if (!error.equals(lastError)) System.err.println("CDO persistence: " + error);
      lastError = error;
    }
  }

  static EPackage isolatedSchema(PureBigraph model) throws Exception {
    String controls =
        model.getSignature().getControls().stream()
            .map(c -> c.getNamedType().stringValue() + ":" + c.getArity().getValue())
            .sorted()
            .collect(java.util.stream.Collectors.joining(";"));
    String digest =
        HexFormat.of()
            .formatHex(
                MessageDigest.getInstance("SHA-256")
                    .digest(controls.getBytes(StandardCharsets.UTF_8)));
    EPackage schema = EcoreUtil.copy(model.getMetaModel());
    schema.setNsURI("urn:cps:cooperative:bigraph:v1:" + digest);
    schema.setNsPrefix("cooperative");
    return schema;
  }

  static EObject storageCopy(PureBigraph model, EPackage schema) {
    EcoreUtil.Copier copier =
        new EcoreUtil.Copier() {
          @Override
          protected EClass getTarget(EClass type) {
            if (type.getEPackage() == model.getMetaModel())
              return (EClass) schema.getEClassifier(type.getName());
            return super.getTarget(type);
          }

          @Override
          protected EStructuralFeature getTarget(EStructuralFeature feature) {
            return getTarget(feature.getEContainingClass())
                .getEStructuralFeature(feature.getName());
          }
        };
    EObject result = copier.copy(model.getInstanceModel());
    copier.copyReferences();
    return result;
  }

  public void close() {
    worker.shutdownNow();
    try {
      worker.awaitTermination(3, TimeUnit.SECONDS);
      if (factory != null) factory.destroy();
    } catch (Exception e) {
      System.err.println("CDO close: " + e.getMessage());
    }
  }
}
