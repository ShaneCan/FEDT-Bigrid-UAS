package org.example;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import org.bigraphs.framework.core.Bigraph;
import org.bigraphs.framework.core.BigraphFileModelManagement;
import org.bigraphs.framework.core.Control;
import org.bigraphs.framework.core.exceptions.IncompatibleSignatureException;
import org.bigraphs.framework.core.exceptions.InvalidConnectionException;
import org.bigraphs.framework.core.exceptions.builder.TypeNotExistsException;
import org.bigraphs.framework.core.exceptions.operations.IncompatibleInterfaceException;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.framework.core.impl.pure.PureBigraphBuilder;
import org.bigraphs.framework.core.impl.signature.DynamicSignature;
import org.bigraphs.framework.core.impl.signature.DynamicSignatureBuilder;
import org.bigraphs.framework.core.reactivesystem.BigraphMatch;
import org.bigraphs.framework.core.reactivesystem.InstantiationMap;
import org.bigraphs.framework.core.reactivesystem.ParametricReactionRule;
import org.bigraphs.framework.core.utils.BigraphUtil;
import org.bigraphs.framework.simulation.matching.AbstractBigraphMatcher;
import org.bigraphs.framework.simulation.matching.pure.PureReactiveSystem;
import org.bigraphs.model.bigraphBaseModel.impl.BigraphBaseModelPackageImpl;
import org.bigraphs.spring.data.cdo.CdoTemplate;
import org.eclipse.emf.cdo.common.id.CDOID;
import org.eclipse.emf.cdo.common.model.CDOPackageRegistry;
import org.eclipse.emf.cdo.util.CDOUtil;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EPackage;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.Import;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import javax.json.JsonObject;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BinaryOperator;

import static org.bigraphs.framework.core.factory.BigraphFactory.*;

@SpringBootApplication
@Import(value = {CDOServerConfig.class})
public class Application3D implements CommandLineRunner {

    // Resource paths
    private static final String WORLD_RESOURCE_BASE = "src/test/resources/models/3Ddiagonaldirectional/";
    private static final String WORLD_SIGNATURE_MM = WORLD_RESOURCE_BASE + "mm_sig_loc.ecore";
    private static final String WORLD_SIGNATURE_INSTANCE = WORLD_RESOURCE_BASE + "sig_loc.xmi";
    
    // Timing constants (milliseconds)
    private static final long TAKEOFF_PROTECTION_MS = 5000;
    private static final long TAKEOFF_COOLDOWN_MS = 5000;
    private static final long WAIT_TIMEOUT_MS = 8000;
    /** Move timeout: if the drone has not reached movingToGrid within this time after the navigation command was sent, stop waiting and replan from the current position */
    private static final long MOVE_TIMEOUT_MS = 10000;
    private static final long MOVE_INTERVAL_MS = 2000;
    private static final long LANDING_DELAY_MS = 2000;
    
    private static double elapsedMs(long startNano, long endNano) {
        return (endNano - startNano) / 1_000_000.0;
    }

    /** Appends a sample to this drone's queue for the given phase store, creating it on first use. */
    private static void recordLatency(Map<String, Queue<Double>> store, String droneId, double ms) {
        store.computeIfAbsent(droneId, k -> new ConcurrentLinkedQueue<>()).add(ms);
    }

    /** A* path planning + diagonal/collision safety check (excludes bigraph match, reservation, REST and sleep) */
    private void logPlanningLatency(String droneId, String action, long startNano, long endNano) {
        double ms = elapsedMs(startNano, endNano);
        recordLatency(planningLatenciesMsByDrone, droneId, ms);
        System.out.println("  ⏱ [Planning] " + droneId + " " + action + ": "
                + String.format(Locale.ROOT, "%.3f", ms) + " ms");
    }

    /** Bigraph match only (no path planning, no CDO persistence) - the decision time of the FEDT runtime gate itself */
    private void logMatchOnlyLatency(String droneId, String action, long startNano, long endNano) {
        double ms = elapsedMs(startNano, endNano);
        recordLatency(gateMatchLatenciesMsByDrone, droneId, ms);
        System.out.println("  ⏱ [Match] " + droneId + " " + action + ": "
                + String.format(Locale.ROOT, "%.3f", ms) + " ms");
    }

    /** REST: from the httpClient.send call until the response arrives (excludes sleep) */
    private void logRestLatency(String droneId, String action, long startNano, long endNano) {
        double ms = elapsedMs(startNano, endNano);
        recordLatency(restLatenciesMsByDrone, droneId, ms);
        System.out.println("  ⏱ [REST] " + droneId + " " + action + ": "
                + String.format(Locale.ROOT, "%.3f", ms) + " ms");
    }
    
    // Altitude thresholds (meters)
    private static final double TAKEOFF_ALTITUDE = 0.1;
    private static final double LANDING_ALTITUDE = 0.1;
    
    // Battery thresholds (volts)
    private static final double BATTERY_EMPTY_THRESHOLD = 3.1; // paper value: 3.1
    private static final double BATTERY_LOW_THRESHOLD = 3.2;   // paper value: 3.2

    // RSSI threshold
    private static final int RSSI_BAD_THRESHOLD = 65; // paper value: 65

    // Synthetic readings for fault injection (chosen relative to the thresholds above; injecting these values triggers the corresponding rule / emergency response)
    private static final double INJECT_BATTERY_NORMAL_V = 4.0;   // >= BATTERY_LOW_THRESHOLD
    private static final double INJECT_BATTERY_LOW_V    = 3.15;  // [EMPTY, LOW) -> Low
    private static final double INJECT_BATTERY_EMPTY_V  = 3.0;  // < EMPTY     -> Empty
    private static final int    INJECT_RSSI_BAD    = 70;         // >= RSSI_BAD_THRESHOLD -> Bad
    private static final int    INJECT_RSSI_NORMAL = 50;         // <  RSSI_BAD_THRESHOLD -> Normal
    @Autowired
    protected CdoTemplate template;

    @Value("${bigrid.service.base-url:http://127.0.0.1:7070}")
    private String bigridServiceBaseUrl;

    @Value("${bigrid.service.rows:5}")
    private int bigridRows;

    @Value("${bigrid.service.cols:5}")
    private int bigridCols;
    
    @Value("${bigrid.service.layers:3}")
    private int bigridLayers;

    @Value("${bigrid.service.format:xml}")
    private String bigridFormat;

    @Value("${bigrid.service.origin.x:-1.0}")
    private double gridOriginX;

    @Value("${bigrid.service.origin.y:-1.0}")
    private double gridOriginY;
    
    @Value("${bigrid.service.origin.z:0.0}")
    private double gridOriginZ;

    @Value("${bigrid.service.step.x:0.5}")
    private double gridStepX;

    @Value("${bigrid.service.step.y:0.5}")
    private double gridStepY;
    
    @Value("${bigrid.service.layer.height:0.6}")
    private double layerHeight; //0.35 in original

    @Value("${drone.count:8}")
    private int configuredDroneCount;

    @Value("${ros.bridge.host:localhost}")
    private String rosBridgeHost;

    @Value("${ros.update.enabled:true}")
    private boolean rosUpdateEnabled;

    // Whether simulation mode is enabled: positions come from /cf_positions_poses (PoseArray) instead of a per-drone /cfXXX/pose
    @Value("${ros.sim.mode:true}")
    private boolean rosSimMode;

    // Drone Control Configuration
    @Value("${drone.control.enabled:true}")
    private boolean droneControlEnabled;
    @Value("${drone.control.base-url:http://127.0.0.1}")
    private String droneControlBaseUrl;
    @Value("${drone.control.start-port:5000}")
    private int droneControlStartPort;

    // Drone target-point configuration (format: "x1,y1;x2,y2;...")
    @Value("${drone.targets:-1,-1.5;1,-1.5;1.5,-1;1.5,1;1,1.5;-1,1.5;-1.5,1;-1.5,-1}")
    private String droneTargetsConfig;
    
    // Charging-station configuration (multiple stations, comma separated, e.g. 3,8,13)
    @Value("${charging.station.grids:3}")
    private String chargingStationGrids;
    
    // Obstacle grid configuration (multiple obstacles, comma separated, e.g. 15,20,25)
    @Value("${obstacle.grids:13,38}")
    private String obstacleGridsConfig;

    @Value("${baseline.mode:false}")
    private boolean baselineMode;

    // Fault-injection HTTP control port: at runtime, curl can put any drone into battery Low/Empty or comm Bad
    @Value("${fault.injection.enabled:true}")
    private boolean faultInjectionEnabled;
    @Value("${fault.injection.port:8090}")
    private int faultInjectionPort;

    // Collective ascent: after take-off, raise all drones to a common working altitude before grid navigation starts.
    @Value("${collective.ascent.enabled:true}")
    private boolean collectiveAscentEnabled;
    // Target absolute working altitude (metres).
    @Value("${collective.ascent.altitude:0.9}")
    private double collectiveAscentAltitude;
    // Timeout for waiting until all drones have taken off (milliseconds)
    @Value("${collective.ascent.takeoff-wait-timeout-ms:8000}")
    private long collectiveAscentTakeoffWaitMs;
    // Settling wait after the ascent command is sent (milliseconds), so status.z converges to the target altitude
    @Value("${collective.ascent.settle-ms:4000}")
    private long collectiveAscentSettleMs;

    // Absolute-altitude mode for vertical navigation: target altitude = base-altitude + target layer * layerHeight, independent of the current physical altitude,
    @Value("${navigation.absolute-altitude.enabled:true}")
    private boolean absoluteAltitudeEnabled;
    // Physical working altitude of layer 0 (metres); used only in absolute-altitude mode.
    @Value("${navigation.base-altitude:0.3}")
    private double navigationBaseAltitude;

    // Whether to fall back to the start cell when sensor drift puts the position outside the grid (grid=-1)
    @Value("${takeoff.allow-out-of-grid:true}")
    private boolean allowOutOfGridPlacement;

    // Debounce threshold for co-occupancy violations: a cell must be occupied by >=2 drones for this many consecutive frames before one real violation is counted.
    @Value("${collision.violation.debounce-frames:3}")
    private int violationDebounceFrames;

    private final HttpClient httpClient = HttpClient.newBuilder()
            .connectTimeout(Duration.ofSeconds(5))
            .build();
    
    // Drone target-point mapping
    private final Map<String, GridPoint> droneTargets = new HashMap<>();
    
    // List of charging-station positions (multiple stations supported)
    private List<GridPoint> chargingStations = new ArrayList<>();
    
    // Set of obstacle grid indices (thread-safe)
    private final Set<Integer> obstacleGrids = ConcurrentHashMap.newKeySet();

    private final ObjectMapper objectMapper = new ObjectMapper();
    
    // Drone position store (thread-safe): droneId -> {x, y, gridIndex}
    private final Map<String, DronePosition> dronePositions = new ConcurrentHashMap<>();
    
    // Drone status store (thread-safe): droneId -> {status, z, hasTakenOff}
    private final Map<String, DroneStatus> droneStatuses = new ConcurrentHashMap<>();
    
    // Collision-risk flag
    private final AtomicBoolean collisionRiskDetected = new AtomicBoolean(false);
    
    // Grid reservation system: prevents several drones from moving into the same grid at once
    private final Map<Integer, GridReservation> gridReservations = new ConcurrentHashMap<>();
    
    // CDO synchronisation lock: prevents concurrent threads from conflicting on the CDO database
    private final Object cdoLock = new Object();

    private DynamicSignature combinedSignature;
    private DynamicSignature serviceWorldSignature;  // Signature extracted from the service world model
    
    // CDO update strategy: keep the object ID for later updates
    private CDOID cdoIdWorld;
    private CDOID cdoIdDrone;
    private CDOID cdoIdComposed;
    
    // Reference to the current bigraph object
    private PureBigraph worldPart;
    private PureBigraph dronePart;
    private PureBigraph composite;
    
    // Flag: whether the first model update has completed
    private final AtomicBoolean firstUpdateCompleted = new AtomicBoolean(false);

    // Mission metrics (Baseline + Bigraph runs use the same ROS pose span for completion time)
    private final AtomicLong coOccupancyViolationCount = new AtomicLong(0);
    // Debounce: counts, per cell, how many consecutive frames it was occupied by >=2 drones; only when the count reaches the threshold is one real violation recorded,
    // which filters out single-frame false overlaps caused by boundary jitter or asynchronous ghosting.
    private final Map<Integer, Integer> coOccupancyStreak = new ConcurrentHashMap<>();
    // Cells already counted during the current sustained-conflict window (cleared so the same cell can be counted again on a new conflict).
    private final Set<Integer> countedViolationGrids = ConcurrentHashMap.newKeySet();

    private final Map<String, Queue<Double>> planningLatenciesMsByDrone = new ConcurrentHashMap<>();
    private final Map<String, Queue<Double>> gateMatchLatenciesMsByDrone = new ConcurrentHashMap<>();
    private final Map<String, Queue<Double>> restLatenciesMsByDrone = new ConcurrentHashMap<>();
    private volatile Long firstRosPoseTimeMs = null;
    private volatile Long lastRosPoseTimeMs = null;
    private volatile Long allDronesLandedAtMs = null;
    // AtomicBoolean (not plain volatile) so the normal completion path and the JVM shutdown-hook
    // path (which can race on Ctrl+C) can never both write the report.
    private final AtomicBoolean missionReportPrinted = new AtomicBoolean(false);

    public static void main(String[] args) {
        BigraphBaseModelPackageImpl.init();
        SpringApplication.run(Application3D.class, args);
    }

    @Override
    public void run(String... args) throws Exception {
        // Ctrl+C (or any JVM shutdown) before all drones land would otherwise silently discard
        // every collected metric (completion time, violations, per-decision latencies), since
        // they only ever lived in memory and were written out by maybePrintMissionReport().
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            if (!missionReportPrinted.get()) {
                System.out.println("\n⚠ JVM shutting down before mission completion — writing partial mission report...");
                writeMissionReport("interrupted");
            }
        }, "MissionReportShutdownHook"));

        TimeUnit.MILLISECONDS.sleep(2500);

        if (baselineMode) {
            System.out.println("\n========================================");
            System.out.println("  BASELINE MODE (A* without Bigraph step check)");
            System.out.println("========================================\n");
        }

        prepareDatabase();

        // Step 1: obtain the world signature (from a local file, compatible with the service)
        fetchWorldSignatureFromService();
        
        // Step 2: build the merged signature (world + drone)
        System.out.println("Building merged signature");
        sig();

        // Step 3: load the world model (using the world signature)
        worldPart = fetchWorldModelFromService();
        
        // Step 4: register the metamodel with CDO
        registerMetaModelToCDO();

        System.out.println("\n========================================");
        System.out.println("Creating Drone Model and Composite Model");
        System.out.println("========================================");
        
        // Step 5: create the drone model and the composite model
        int currentSiteCount = worldPart.getSites().size();
        dronePart = droneModel(currentSiteCount);
        composite = composeWorldAndDrones(worldPart, dronePart);

        // Step 6: insert the object and keep its CDOID for later updates (history is retained); this needs metamodel information, so the metamodel must be registered first (step 4)
        EPackage MM = createOrGetBigraphMetaModel(sig());
        worldPart = BigraphUtil.toBigraph(MM, template.insert(worldPart.getInstanceModel(), "/world"), sig());
        dronePart = BigraphUtil.toBigraph(MM, template.insert(dronePart.getInstanceModel(), "/drone"), sig());
        composite = BigraphUtil.toBigraph(MM, template.insert(composite.getInstanceModel(), "/composed"), sig());

        // Step 7: store the CDOID for later updates
        cdoIdWorld = CDOUtil.getCDOObject(worldPart.getInstanceModel()).cdoID();
        cdoIdDrone = CDOUtil.getCDOObject(dronePart.getInstanceModel()).cdoID();
        cdoIdComposed = CDOUtil.getCDOObject(composite.getInstanceModel()).cdoID();
        
        System.out.println("✓ World Model inserted into CDO (CDOID: " + cdoIdWorld + ")");
        System.out.println("✓ Drone Model inserted into CDO (CDOID: " + cdoIdDrone + ")");
        System.out.println("✓ Composite Model inserted into CDO (CDOID: " + cdoIdComposed + ")");
        
        // Initialise the ROS2 subscriptions (if enabled)
        initializeRosSubscriptions();

        // Start the fault-injection HTTP endpoint (curl can put any drone into battery Low/Empty or comm Bad at any time)
        startFaultInjectionServer();

        if (!baselineMode) {
            // Start the background model-update thread (continuous fast updates)
            Thread modelUpdateThread = new Thread(this::continuousModelUpdate, "ModelUpdateThread");
            modelUpdateThread.setDaemon(false);
            modelUpdateThread.start();
            
            // Wait for the first model update to complete
            System.out.println("Waiting for first model update to complete...");
            while (!firstUpdateCompleted.get()) {
                TimeUnit.MILLISECONDS.sleep(100);
            }
            System.out.println("✓ First model update completed\n");
        } else {
            firstUpdateCompleted.set(true);
            System.out.println("✓ Baseline mode: skipping continuous Bigraph model update thread\n");
        }
        
        // Start the battery and communication monitoring thread (if ROS2 is enabled)
        if (rosUpdateEnabled) {
            Thread statusMonitorThread = new Thread(this::continuousStatusMonitoring, "StatusMonitorThread");
            statusMonitorThread.setDaemon(false);
            statusMonitorThread.start();
            System.out.println("✓ Battery and communication status monitoring thread started");
        }
        
        // Wait for the ROS2 status data to stabilise (battery voltage, RSSI, etc.)
        // This ensures the pre-take-off check sees the real battery state
        if (rosUpdateEnabled) {
            System.out.println("Waiting for ROS2 status data to stabilize (3 seconds)...");
            TimeUnit.MILLISECONDS.sleep(3000);
            System.out.println("✓ ROS2 status data should be stable now\n");
        }
        
        // Parse the target configuration (after the ROS2 data has stabilised, so start positions are correct)
        parseDroneTargets();
        
        // Run the take-off sequence: match the rule and send the take-off command
        if (droneControlEnabled) {
            performTakeoffSequence();
            // After take-off, collectively ascend to the common working altitude (if enabled) before entering the navigation loop
            performCollectiveAscent();
        }
        
        System.out.println("\n========================================");
        System.out.println("Entering navigation control loop...");
        System.out.println("========================================\n");
        
        // Main control loop: applies the take-off and landing rules
        while (true) {
            TimeUnit.MILLISECONDS.sleep(1000);  // Check once per second
            
            if (!droneControlEnabled || !rosUpdateEnabled) {
                continue;
            }
            
            // Checks and applies the take-off rule once the drone has physically taken off
            checkAndApplyTakeoffRules();
            
            // Run navigation control (plan a path and move)
            if (baselineMode) {
                performBaselineNavigationControl();
            } else {
                performNavigationControl();
            }
            
            // Check and apply the landing rule (land after reaching the target)
            checkAndApplyLandingRules();
            
            maybePrintMissionReport();
        }
    }
    
    /**
     * Continuously updates the model at a fast rate (background thread)
     * ContinuousModelUpdate takes a synchronisation lock so it does not conflict with StatusMonitorThread on CDO
     */
    private void continuousModelUpdate() {
        try {
            int currentSiteCount = worldPart.getSites().size();
            int updateCount = 0;
            
            while (true) {
                updateCount++;
                
                // Fetch the latest world model (no lock needed, this is just an HTTP request)
                PureBigraph latestWorld = fetchWorldModelFromService();
                int newSiteCount = latestWorld.getSites().size();
                
                // Rebuild the drone model when the ROS2 positions or the site count change
                boolean needUpdateDrone = (newSiteCount != currentSiteCount) || 
                                         (rosUpdateEnabled && !dronePositions.isEmpty() && !collisionRiskDetected.get());
                
                //Wrap all CDO operations in the synchronisation lock
                synchronized (cdoLock) {
                    if (needUpdateDrone) {
                        if (newSiteCount != currentSiteCount) {
                            System.out.println("  [Update" + updateCount + "] Site count changed: " + currentSiteCount + " -> " + newSiteCount);
                            currentSiteCount = newSiteCount;
                        }
                        
                        // Build the drone model from the ROS2 positions (if enabled and no collision risk)
                        if (rosUpdateEnabled && !dronePositions.isEmpty() && !collisionRiskDetected.get()) {
                            dronePart = droneModelFromRosPositions(newSiteCount);
                        } else {
                            dronePart = droneModel(newSiteCount);
                        }
                        
                        // Update the drone model: insert a new version (CDO auditing keeps the history)
                        EObject insertedDrone = template.insert(dronePart.getInstanceModel(), "/drone");
                        dronePart = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedDrone, sig());
                        cdoIdDrone = CDOUtil.getCDOObject(dronePart.getInstanceModel()).cdoID();
                    }

                    // Update the world model: insert a new version (CDO auditing keeps the history)
                    EObject insertedWorld = template.insert(latestWorld.getInstanceModel(), "/world");
                    worldPart = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedWorld, sig());
                    cdoIdWorld = CDOUtil.getCDOObject(worldPart.getInstanceModel()).cdoID();

                    // Update the composite model: insert a new version (CDO auditing keeps the history)
                    PureBigraph updatedComposite = composeWorldAndDrones(worldPart, dronePart);
                    EObject insertedComposite = template.insert(updatedComposite.getInstanceModel(), "/composed");
                    composite = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedComposite, sig());
                    cdoIdComposed = CDOUtil.getCDOObject(composite.getInstanceModel()).cdoID();
                }
                
                // Mark the first update as complete
                if (!firstUpdateCompleted.get()) {
                    firstUpdateCompleted.set(true);
                }
                
                // ############## Fixed delay between model updates, in milliseconds.
                TimeUnit.MILLISECONDS.sleep(500);
            }
        } catch (Exception e) {
            System.err.println("!! Model update thread exception: " + e.getMessage());
            e.printStackTrace();
        }
    }
    

    private void prepareDatabase() throws Exception {
        System.out.println("Preparing CDO database...");
        
        // Note: the signature is not built yet; this only cleans the database, metamodel registration happens later
        
        // Delete resources safely: catch every exception to handle dirty or missing resources
        String[] paths = {"/drone", "/world", "/composed"};
        for (String path : paths) {
            try {
                template.removeAll(path);
                System.out.println("  ✓ Cleared path: " + path);
            } catch (org.eclipse.emf.cdo.view.CDOViewSet.CDOViewSetException e) {
                // CDO dirty-resource error: the resource has uncommitted changes; ignore it (likely left over from a previous run)
                System.out.println("  ⚠ Skipped dirty resource: " + path + " (will be overwritten)");
            } catch (Exception e) {
                // Other errors (such as a missing resource): ignore
                System.out.println("  ! Error cleaning " + path + " (ignorable): " + e.getClass().getSimpleName());
            }
        }
        
        System.out.println("✓ CDO database ready");
    }
    
    /**
     * // Step 4: register the metamodel with CDO (preparing for step 6, which inserts the instance model)
     */
    private void registerMetaModelToCDO() throws Exception {
        //System.out.println("Registering the metamodel with CDO...");
        
        DynamicSignature signature = sig();
        EPackage metaModel = createOrGetBigraphMetaModel(signature); // Create or fetch the EMF metamodel corresponding to the signature

        EPackage.Registry.INSTANCE.put(metaModel.getNsURI(), metaModel); // Register the metamodel in the EMF EPackage registry so EMF knows about it
        CDOPackageRegistry.INSTANCE.put(metaModel.getNsURI(), metaModel); // Register the metamodel in the CDO package registry so CDO knows about it
        template.getCDOPackageRegistry().put(metaModel.getNsURI(), metaModel); // Register the metamodel in the Spring Data CDO package registry so this CDO template knows about it
    }

    private DynamicSignature sig() {
        if (combinedSignature == null) {
            combinedSignature = buildCombinedSignature();
        }
        return combinedSignature;
    }

    private DynamicSignature buildCombinedSignature() {
        try {
            System.out.println("Building merged signature (World + Drone)...");
            
            // Make sure the world signature has been loaded
            if (serviceWorldSignature == null) {
                throw new IllegalStateException("World signature not loaded yet!");
            }
            
            // Merge using BigraphUtil.mergeSignatures (same as DroneLandingSystem4x5.java)
            DynamicSignature droneSignature = createDroneSignature();
            DynamicSignature merged = BigraphUtil.mergeSignatures(serviceWorldSignature, droneSignature);
            
            System.out.println("✓ Merged signature built successfully");
            System.out.println("  World signature controls: " + serviceWorldSignature.getControls().size());
            System.out.println("  Drone signature controls: " + droneSignature.getControls().size());
            System.out.println("  Merged total controls: " + merged.getControls().size());
            
            return merged;
        } catch (Exception e) {
            System.err.println("Error building merged signature: " + e.getMessage());
            e.printStackTrace();
            throw new IllegalStateException("Unable to build combined signature", e);
        }
    }

    /**
     * Creates the drone signature
     * Note: uses .add() rather than .addControl() because of a version difference
     */
    private DynamicSignature createDroneSignature() {
        DynamicSignatureBuilder builder = pureSignatureBuilder();
        builder.add("OccupiedBy", 0)
                .add("Bad", 0)
                .add("Drone", 1)
                .add("ID", 0)
                .add("Status", 0)
                .add("Battery", 0)
                .add("Normal", 0)
                .add("Communication", 0)
                .add("Low", 0)
                .add("Landed", 0)
                .add("flying", 0)
                .add("Empty", 0)
                .add("ObstacleOrWeather", 0);
        for (int i = 0; i <= 10; i++) {
            builder.add("D" + i, 0);
        }
        return builder.create();
    }

    /**
     * Loads the world model signature from the local file
     */
    private DynamicSignature fetchWorldSignatureFromService() throws Exception {
        if (serviceWorldSignature != null) {
            return serviceWorldSignature;
        }
        
        System.out.println("\n==================Local World Signature======================");
        
        // Load from the local signature file
            List<EObject> sigObjects = BigraphFileModelManagement.Load.signatureInstanceModel(
                    WORLD_SIGNATURE_MM, WORLD_SIGNATURE_INSTANCE);
        serviceWorldSignature = (DynamicSignature) createOrGetSignature(sigObjects.get(0));
        
        System.out.println("  Control count: " + serviceWorldSignature.getControls().size());
        System.out.println("  Control list: ");
        for (Control<?, ?> control : serviceWorldSignature.getControls()) {
            System.out.println("    - " + control.getNamedType().stringValue() + 
                             " (arity: " + control.getArity().getValue() + ")");
        }
        System.out.println("========================================\n");
        
        return serviceWorldSignature;
    }

    private PureBigraph fetchWorldModelFromService() throws Exception {
        //System.out.println("\n===========Fetching the world model instance=============================");
        
        // Important: create the bigraph metamodel from the merged signature
        DynamicSignature combinedSig = sig();
        EPackage metaModel = createOrGetBigraphMetaModel(combinedSig);
        
        //System.out.println("  Signature control count: " + combinedSig.getControls().size());
        
        System.out.println("\nFetching 3D BiGrid instance model...");
        URI uri = URI.create(String.format(Locale.ROOT,
                "%s/generate/3d-diagonal-directional/bigrid?rows=%d&cols=%d&layers=%d&format=%s",
                bigridServiceBaseUrl, bigridRows, bigridCols, bigridLayers, bigridFormat));
        
        HttpRequest request = HttpRequest.newBuilder(uri)
                .timeout(Duration.ofSeconds(10))
                .header("Content-Type", "application/json")
                .POST(HttpRequest.BodyPublishers.ofString(buildGridRequestPayload(), StandardCharsets.UTF_8))
                .build();

        HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString(StandardCharsets.UTF_8));
        
        if (response.statusCode() != 200) {
            throw new IllegalStateException("Failed to fetch BiGrid model, status=" + response.statusCode());
        }

        // Extract the XML content from the JSON response
        String xmlContent = extractXmlFromJsonResponse(response.body());
        
        // Deserialise the instance model using the merged-signature metamodel
        try (ByteArrayInputStream inputStream = new ByteArrayInputStream(xmlContent.getBytes(StandardCharsets.UTF_8))) {
            // Load the instance using the merged-signature metamodel
            List<EObject> worldObjects = BigraphFileModelManagement.Load.bigraphInstanceModel(metaModel, inputStream);
            
            // Convert to a bigraph using the merged signature
            PureBigraph bigraph = BigraphUtil.toBigraph(metaModel, worldObjects.get(0), combinedSig);
            //System.out.println("  Deserialisation succeeded, site count: " + bigraph.getSites().size());
            //System.out.println("========================================\n");
            
            return bigraph;
        } catch (Exception e) {
            System.err.println("!!! Error loading World Model !!!");
            System.err.println("Error type: " + e.getClass().getName());
            System.err.println("Error message: " + e.getMessage());
            e.printStackTrace();
            throw e;
        }
    }

    /**
     * Extract the XML content from the JSON response
     */
    private String extractXmlFromJsonResponse(String responseBody) throws Exception {
        String trimmedBody = responseBody.trim();
        
        try {
            // Check whether the payload is JSON
            if (trimmedBody.startsWith("{")) {
                JsonNode jsonNode = objectMapper.readTree(trimmedBody);
                
                // Check for a content field
                if (jsonNode.has("content")) {
                    String content = jsonNode.get("content").asText();
                    
                    // Check the mimeType
                    String mimeType = jsonNode.has("mimeType") ? jsonNode.get("mimeType").asText() : "";
                    
                    // If the mimeType says XML, return the content directly
                    if (mimeType.contains("xml")) {
                        //System.out.println("Extracting XML content from the JSON wrapper");
                        return content;
                    }
                    
                    // Without an explicit XML mimeType, try to detect whether the content is XML
                    if (content.trim().startsWith("<?xml") || content.trim().startsWith("<")) {
                        //System.out.println("The content looks like XML, using it directly");
                        return content;
                    }
                }
                
                throw new IllegalStateException("The JSON response contains no valid XML content");
            }
            
            // If it is not JSON, assume plain XML
            if (trimmedBody.startsWith("<?xml") || trimmedBody.startsWith("<")) {
                System.out.println("Response appears to be pure XML");
                return trimmedBody;
            }
            
            throw new IllegalStateException("The response is neither valid JSON nor XML. First 100 characters: " + 
                    trimmedBody.substring(0, Math.min(100, trimmedBody.length())));
            
        } catch (Exception e) {
            System.err.println("Error parsing response: " + e.getMessage());
            System.err.println("Response preview (first 500 chars): " + 
                    trimmedBody.substring(0, Math.min(500, trimmedBody.length())));
            throw e;
        }
    }

    private String buildGridRequestPayload() {
        return String.format(Locale.ROOT,
                "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"stepSizeX\":%.3f,\"stepSizeY\":%.3f,\"layerHeight\":%.3f}",
                gridOriginX, gridOriginY, gridOriginZ, gridStepX, gridStepY, layerHeight);
    }

    // Used for the initial placement of the drone model
    private PureBigraph droneModel(int siteCount) throws InvalidConnectionException, TypeNotExistsException {
        if (siteCount <= 0) {
            return pureBuilder(sig()).create();
        }

        List<Bigraph<DynamicSignature>> placements = new ArrayList<>();
        for (int i = 0; i < siteCount; i++) {
            // Check whether this is an obstacle grid
            if (obstacleGrids.contains(i)) {
                placements.add(buildObstacleCell());
            } else {
                placements.add(emptyOccupiedCell()); 
            }
        }

        int dronesToPlace = Math.min(configuredDroneCount, siteCount);
        for (int idx = 0; idx < dronesToPlace; idx++) {
            // Skip obstacle grids; do not place a drone on an obstacle
            if (obstacleGrids.contains(idx)) {
                continue;
            }
            
            String droneId = "D" + idx;
            // Get the actual drone status
            String droneStatus = "Landed";  // Default status
            DroneStatus status = droneStatuses.get(droneId);
            if (status != null) {
                if (status.landingRuleApplied) {
                    droneStatus = "Landed";  // The landing rule has been applied, status is Landed
                } else if (status.takeoffRuleApplied) {
                    droneStatus = "flying";  // The take-off rule has been applied, status is flying
                }
            }
            placements.set(idx, buildDrone(droneId, droneStatus, "OccupiedBy"));
        }

        Bigraph<DynamicSignature> result = placements.stream()
                .reduce(pureLinkings(sig()).identity_e(), accumulator::apply); // pureLinkings(sig()).identity_e() is an empty bigraph
        return (PureBigraph) result;
    }

    private PureBigraph buildDrone(String id, String status, String nodeType) throws InvalidConnectionException, TypeNotExistsException {
        PureBigraphBuilder<DynamicSignature> builder = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        
        // Read the in-memory battery and communication state of the drone, used to build the drone model each round
        DroneStatus droneStatus = droneStatuses.get(id);
        String batteryLevel = (droneStatus != null) ? droneStatus.batteryLevel : "Normal";
        String communicationStatus = (droneStatus != null) ? droneStatus.communicationStatus : "Normal";

        return builder.root()
                .child(nodeType).down()
                .child("Drone", normalizedId).down()
                .child("ID").down().child(id).up()
                .child("Status").down().child(status).up()
                .child("Battery").down().child(batteryLevel).up()
                .child("Communication").down().child(communicationStatus).up()
                .create();
    }

    private PureBigraph emptyOccupiedCell() throws InvalidConnectionException, TypeNotExistsException {
        PureBigraphBuilder<DynamicSignature> builder = pureBuilder(sig());
        return builder.root()
                .child("OccupiedBy")
                .create();
    }
    
    /**
     * Builds an OccupiedBy node containing an obstacle
     * The OccupiedBy node contains an ObstacleOrWeather node, marking the cell as occupied by an obstacle
     */
    private PureBigraph buildObstacleCell() throws InvalidConnectionException, TypeNotExistsException {
        PureBigraphBuilder<DynamicSignature> builder = pureBuilder(sig());
        return builder.root()
                .child("OccupiedBy").down()
                .child("ObstacleOrWeather")
                .create();
    }

    private PureBigraph composeWorldAndDrones(PureBigraph world, PureBigraph drones) throws IncompatibleSignatureException, IncompatibleInterfaceException {
        return ops(world).nesting(drones).getOuterBigraph();
    }

    // Accumulator for merging a list of bigraphs; parallelProduct places two bigraphs side by side
    private final BinaryOperator<Bigraph<DynamicSignature>> accumulator = (partial, element) -> {
        try {
            return ops(partial).parallelProduct(element).getOuterBigraph();
        } catch (IncompatibleSignatureException | IncompatibleInterfaceException e) {
            return pureLinkings(partial.getSignature()).identity_e();
        }
    };

    private ParametricReactionRule<PureBigraph> droneTakeOffRule(String id) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        redexB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("Landed").up()
                .child("Battery").down()
                .child("Normal").up()
                .child("Communication").down()
                .child("Normal").up()
        ;

        reactumB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying").up()
                .child("Battery").down()
                .child("Normal").up()
                .child("Communication").down()
                .child("Normal").up()
        ;
        InstantiationMap instantiationMap = InstantiationMap.create(1);
        instantiationMap.map(0, 0);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
    }
    
    private ParametricReactionRule<PureBigraph> droneLandingRule(String id) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        redexB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying");

        reactumB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("Landed");
        InstantiationMap instantiationMap = InstantiationMap.create(2);
        instantiationMap.map(0, 0);
        instantiationMap.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
    }

    /** Generic battery state-transition rule */
    private ParametricReactionRule<PureBigraph> createBatteryRule(String id, String from, String to) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        
        redexB.root().child("OccupiedBy").down().site()
                .child("Drone", normalizedId).down().site()
                .child("ID").down().child(id).up()
                .child("Battery").down().child(from);
        reactumB.root().child("OccupiedBy").down().site()
                .child("Drone", normalizedId).down().site()
                .child("ID").down().child(id).up()
                .child("Battery").down().child(to);
        
        InstantiationMap map = InstantiationMap.create(2);
        map.map(0, 0);
        map.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), map);
    }
    
    private ParametricReactionRule<PureBigraph> droneBatteryNormalToLowRule(String id) throws Exception {
        return createBatteryRule(id, "Normal", "Low");
    }
    
    private ParametricReactionRule<PureBigraph> droneBatteryLowToNormalRule(String id) throws Exception {
        return createBatteryRule(id, "Low", "Normal");
    }
    
    private ParametricReactionRule<PureBigraph> droneBatteryEmptyToLowRule(String id) throws Exception {
        return createBatteryRule(id, "Empty", "Low");
    }
    
    private ParametricReactionRule<PureBigraph> droneBatteryLowToEmptyRule(String id) throws Exception {
        return createBatteryRule(id, "Low", "Empty");
    }

    /** Generic communication state-transition rule */
    private ParametricReactionRule<PureBigraph> createCommunicationRule(String id, String from, String to) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        
        redexB.root().child("OccupiedBy").down().site()
                .child("Drone", normalizedId).down().site()
                .child("ID").down().child(id).up()
                .child("Communication").down().child(from);
        reactumB.root().child("OccupiedBy").down().site()
                .child("Drone", normalizedId).down().site()
                .child("ID").down().child(id).up()
                .child("Communication").down().child(to);
        
        InstantiationMap map = InstantiationMap.create(2);
        map.map(0, 0);
        map.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), map);
    }
    
    private ParametricReactionRule<PureBigraph> droneCommunicationNormalToBadRule(String id) throws Exception {
        return createCommunicationRule(id, "Normal", "Bad");
    }
    
    private ParametricReactionRule<PureBigraph> droneCommunicationBadToNormalRule(String id) throws Exception {
        return createCommunicationRule(id, "Bad", "Normal");
    }


    /**
     * Creates a directional movement rule (generic helper)
     * @param id drone ID
     * @param routeType route type (ForwardRoute, BackRoute, LeftRoute, RightRoute)
     * @return the movement rule
     */
    private ParametricReactionRule<PureBigraph> createDirectionalMoveRule(String id, String routeType) throws Exception {
        PureBigraphBuilder<DynamicSignature> redexB = pureBuilder(sig());
        PureBigraphBuilder<DynamicSignature> reactumB = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        
        redexB.root()
                .child("Locale", "occupied1").down()
                .site()
                .child(routeType, "occupied2")
                .child("OccupiedBy").down()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying")
                .top()
                .child("Locale", "occupied2").down()
                .site()
                .child("OccupiedBy");

        reactumB.root()
                .child("Locale", "occupied1").down()
                .site()
                .child(routeType, "occupied2")
                .child("OccupiedBy")
                .top()
                .child("Locale", "occupied2").down()
                .site()
                .child("OccupiedBy").down()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying");
        
        InstantiationMap instantiationMap = InstantiationMap.create(3);
        instantiationMap.map(0, 0);
        instantiationMap.map(1, 1);
        instantiationMap.map(2, 2);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
    }
    
    /**
     * Sends an HTTP POST request to the drone control service
     * @param port port number
     * @param endpoint API endpoint (e.g. "/activate_idle", "/begin_takeoff")
     * @param successMessage message logged on success
     * @param errorPrefix prefix for the error message
     * @return whether the request succeeded
     */
    private boolean sendDroneControlRequest(String droneId, int port, String endpoint, String successMessage, String errorPrefix) {
        try {
            String url = droneControlBaseUrl + ":" + port + endpoint;
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofSeconds(5))
                    .POST(HttpRequest.BodyPublishers.noBody())
                    .build();
            
            long restStart = System.nanoTime();
            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
            long restEnd = System.nanoTime();
            
            if (response.statusCode() == 200) {
                System.out.println("  ✓ " + droneId + " " + successMessage);
                logRestLatency(droneId, endpoint, restStart, restEnd);
                return true;
            } else {
                System.err.println("  !! " + droneId + " " + errorPrefix + " failed: HTTP " + response.statusCode());
                return false;
            }
        } catch (Exception e) {
            System.err.println("  !! " + droneId + " " + errorPrefix + " failed: " + e.getMessage());
            return false;
        }
    }
    
    /**
     * Sends a drone control command: activate the idle state
     */
    private boolean activateIdle(String droneId, int port) {
        return sendDroneControlRequest(droneId, port, "/activate_idle", "Idle state has been activated.", "activate idle");
    }
    
    /**
     * Sends a drone control command: begin take-off
     */
    private boolean beginTakeoff(String droneId, int port) {
        return sendDroneControlRequest(droneId, port, "/begin_takeoff", "Takeoff command has been transmitted.", "takeoff");
    }
    
    /**
     * Sends a drone navigation command: move to the given position
     */
    private boolean navigateTo(String droneId, int port, double x, double y, double z) {
        try {
            String url = droneControlBaseUrl + ":" + port + "/navigate/" + x + "/" + y + "/" + z;
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofSeconds(5))
                    .POST(HttpRequest.BodyPublishers.noBody())
                    .build();
            
            long restStart = System.nanoTime();
            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
            long restEnd = System.nanoTime();
            
            if (response.statusCode() == 200) {
                System.out.println("  ✓ " + droneId + " navigation command sent -> (" + 
                        String.format("%.2f, %.2f, %.2f", x, y, z) + ")");
                logRestLatency(droneId, "navigate", restStart, restEnd);
                return true;
            } else {
                System.err.println("  !! " + droneId + " navigation failed: HTTP " + response.statusCode());
                return false;
            }
        } catch (Exception e) {
            System.err.println("  !! " + droneId + " navigation failed: " + e.getMessage());
            return false;
        }
    }
    
    /**
     * Sends a drone control command: begin landing
     */
    private boolean beginLanding(String droneId, int port) {
        return sendDroneControlRequest(droneId, port, "/begin_landing", "Landing command has been transmitted.", "landing");
    }
    
    /**
     * Runs the take-off sequence for every drone: match the rule, then send the take-off command
     * Uses the current composite field rather than the argument, so the latest state is seen after the battery/comm rules have been applied
     */
    private void performTakeoffSequence() throws Exception {
        if (!droneControlEnabled) {
            System.out.println("\n========================================");
            System.out.println("Drone control function not enabled");
            System.out.println("To enable, set: drone.control.enabled=true");
            System.out.println("========================================\n");
            return;
        }
        
        System.out.println("\n========================================");
        System.out.println(baselineMode
                ? "Starting takeoff sequence: direct REST control (no Bigraph match)"
                : "Starting takeoff sequence: rule matching and takeoff control");
        System.out.println("========================================");
        
        if (!baselineMode) {
            // Apply the battery and communication rules once before take-off so the bigraph state is up to date
            System.out.println("\n[Pre-Takeoff Check] Applying battery and communication rules...");
            for (int i = 0; i < configuredDroneCount; i++) {
                String droneId = "D" + i;
                DroneStatus status = droneStatuses.get(droneId);
                if (status != null) {
                    try {
                        System.out.println("  " + droneId + " battery voltage: " + 
                                String.format("%.2f", status.batteryVoltage) + "V, level: " + status.batteryLevel);
                        checkAndApplyBatteryRule(droneId, status);
                        checkAndApplyCommunicationRule(droneId, status);
                    } catch (Exception e) {
                        System.err.println("  !! Error applying rules for " + droneId + ": " + e.getMessage());
                    }
                }
            }
            System.out.println("✓ Pre-takeoff status check completed\n");
        }
        
        AbstractBigraphMatcher<PureBigraph> matcher = baselineMode ? null
                : AbstractBigraphMatcher.create(PureBigraph.class);
        
        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            int cfNumber = 231 + i;
            int port = droneControlStartPort + i;
            
            try {
                System.out.println("\nProcessing " + droneId + " (cf" + cfNumber + ", port " + port + "):");
                
                // Log the current battery state
                DroneStatus status = droneStatuses.get(droneId);
                if (status != null) {
                    System.out.println("  Current status - Battery: " + status.batteryLevel + 
                            " (" + String.format("%.2f", status.batteryVoltage) + "V), Comm: " + status.communicationStatus);
                }
                
                // Create the take-off rule
                ParametricReactionRule<PureBigraph> takeoffRule = baselineMode ? null : droneTakeOffRule(droneId);
                
                boolean takeoffMatched = baselineMode;
                if (!baselineMode) {
                    long takeoffMatchStart = System.nanoTime();
                    Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(composite, takeoffRule).iterator();
                    takeoffMatched = matchIterator.hasNext();
                    long takeoffMatchEnd = System.nanoTime();
                    if (takeoffMatched) {
                        logMatchOnlyLatency(droneId, "takeoff authorization", takeoffMatchStart, takeoffMatchEnd);
                    }
                }
                
                if (takeoffMatched) {
                    if (!baselineMode) {
                        System.out.println("  ✓ Rule matched successfully - " + droneId + " can take off");
                    } else {
                        System.out.println("  ✓ Baseline takeoff - " + droneId + " (no Bigraph check)");
                    }
                    
                    System.out.println("  → Sending takeoff commands...");
                    
                    if (!activateIdle(droneId, port)) {
                        System.err.println("  !! Unable to activate idle state, skipping " + droneId);
                        continue;
                    }
                    
                    TimeUnit.MILLISECONDS.sleep(500);  // Wait for the state to settle (not counted in the latency above)
                    
                    if (!beginTakeoff(droneId, port)) {
                        System.err.println("  !! Unable to send takeoff command, skipping " + droneId);
                        continue;
                    }
                    
                    // Battery protection window: keeps the battery state unchanged for five seconds before take-off; takeoffTime is set as soon as the take-off command is sent
                    if (status != null) {
                        status.takeoffTime = System.currentTimeMillis();
                        status.takeoffCommandSent = true;  // Mark the take-off command as sent
                    }
                    
                    System.out.println("  ✓ " + droneId + " takeoff sequence initiated");
                } else {
                    // Log the detailed reason the rule did not match
                    String reason = "";
                    if (status != null) {
                        if (!"Normal".equals(status.batteryLevel)) {
                            reason = "Battery is " + status.batteryLevel + " (requires Normal)";
                        } else if (!"Normal".equals(status.communicationStatus)) {
                            reason = "Communication is " + status.communicationStatus + " (requires Normal)";
                        } else if (status.hasTakenOff || status.takeoffRuleApplied) {
                            reason = "already in flying status";
                        } else {
                            reason = "unknown (check Bigraph model)";
                        }
                    }
                    System.out.println("  ✗ Rule not matched - " + droneId + " cannot take off: " + reason);
                }
            } catch (Exception e) {
                System.err.println("  !! Error processing " + droneId + ": " + e.getMessage());
                e.printStackTrace();
            }
        }
        
        System.out.println("\n========================================");
        System.out.println("Takeoff sequence completed");
        System.out.println("========================================\n");
    }

    /**
     * Collective ascent: after take-off, raise every drone to {@code collectiveAscentAltitude} before entering the navigation loop.
     * Toggled with collective.ascent.enabled; collective.ascent.altitude sets the initial working altitude.
     * Steps: 1) wait until every drone reports hasTakenOff; 2) send an ascent command to each drone, keeping its horizontal position; 3) wait for the state to settle.
     * Note: horizontal moves keep the current altitude and vertical moves change it by layerHeight, so the raised baseline altitude is preserved by later navigation.
     */
    private void performCollectiveAscent() {
        if (!collectiveAscentEnabled) {
            return;
        }
        if (!droneControlEnabled) {
            System.out.println("[Collective Ascent] Skipped: drone control not enabled");
            return;
        }
        if (!rosUpdateEnabled) {
            System.out.println("[Collective Ascent] Skipped: ROS updates disabled (no altitude feedback)");
            return;
        }

        System.out.println("\n========================================");
        System.out.println("[Collective Ascent] Raising all UAS to operating altitude "
                + String.format(Locale.ROOT, "%.2f", collectiveAscentAltitude) + "m");
        System.out.println("========================================");

        // 1) Wait until every drone (excluding those on obstacle cells) has taken off
        long waitStart = System.currentTimeMillis();
        while (true) {
            int taken = 0, active = 0;
            for (int i = 0; i < configuredDroneCount; i++) {
                if (obstacleGrids.contains(i)) {
                    continue; // No drone is placed on an obstacle cell
                }
                active++;
                DroneStatus st = droneStatuses.get("D" + i);
                if (st != null && st.hasTakenOff) {
                    taken++;
                }
            }
            if (active > 0 && taken >= active) {
                System.out.println("  ✓ All " + active + " UAS airborne, beginning collective ascent");
                break;
            }
            if (System.currentTimeMillis() - waitStart > collectiveAscentTakeoffWaitMs) {
                System.out.println("  ⚠ Timeout waiting for takeoff (" + taken + "/" + active
                        + " airborne); proceeding with those airborne");
                break;
            }
            try {
                TimeUnit.MILLISECONDS.sleep(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

        // 2) Send an ascent command per drone: keep the current x,y and only raise the altitude
        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            DroneStatus st = droneStatuses.get(droneId);
            if (st == null || !st.hasTakenOff) {
                continue;
            }
            DronePosition pos = dronePositions.get(droneId);
            if (pos == null) {
                System.out.println("  ⚠ " + droneId + " has no position yet, skipping ascent");
                continue;
            }
            int port = droneControlStartPort + i;
            System.out.println("  ↑ " + droneId + " ascending from "
                    + String.format(Locale.ROOT, "%.2f", st.z) + "m to "
                    + String.format(Locale.ROOT, "%.2f", collectiveAscentAltitude) + "m");
            navigateTo(droneId, port, pos.x, pos.y, collectiveAscentAltitude);
        }

        // 3) Wait for status.z to converge to the target altitude before starting navigation
        try {
            TimeUnit.MILLISECONDS.sleep(collectiveAscentSettleMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return;
        }

        System.out.println("[Collective Ascent] Completed. UAS now operating at ~"
                + String.format(Locale.ROOT, "%.2f", collectiveAscentAltitude) + "m");
        System.out.println("========================================\n");
    }

    /** Movement directions: 8 horizontal + 2 vertical */
    private enum MoveDirection {
        FORWARD, BACK, LEFT, RIGHT, FORWARD_LEFT, FORWARD_RIGHT, BACK_LEFT, BACK_RIGHT, UP, DOWN
    }
    
    private MoveDirection getMoveDirection(GridPoint current, GridPoint next) {
        double dx = next.x - current.x, dy = next.y - current.y, dz = next.z - current.z;
        boolean zChg = Math.abs(dz) > 0.01, xChg = Math.abs(dx) > 0.01, yChg = Math.abs(dy) > 0.01;
        
        if (zChg && !xChg && !yChg) return dz > 0 ? MoveDirection.UP : MoveDirection.DOWN;
        if (xChg && yChg) {
            if (dx > 0 && dy > 0) return MoveDirection.FORWARD_LEFT;
            if (dx > 0 && dy < 0) return MoveDirection.FORWARD_RIGHT;
            if (dx < 0 && dy > 0) return MoveDirection.BACK_LEFT;
            return MoveDirection.BACK_RIGHT;
        }
        if (xChg) return dx > 0 ? MoveDirection.FORWARD : MoveDirection.BACK;
        if (yChg) return dy > 0 ? MoveDirection.LEFT : MoveDirection.RIGHT;
        return MoveDirection.FORWARD;
    }
    
    private ParametricReactionRule<PureBigraph> getMoveRuleForDirection(String droneId, MoveDirection dir) throws Exception {
        String routeType = switch (dir) {
            case FORWARD -> "ForwardRoute";
            case BACK -> "BackRoute";
            case LEFT -> "LeftRoute";
            case RIGHT -> "RightRoute";
            case FORWARD_LEFT -> "ForwardLeftRoute";
            case FORWARD_RIGHT -> "ForwardRightRoute";
            case BACK_LEFT -> "BackLeftRoute";
            case BACK_RIGHT -> "BackRightRoute";
            case UP -> "UpRoute";
            case DOWN -> "DownRoute";
        };
        return createDirectionalMoveRule(droneId, routeType);
    }

    /**
     * Computes the target altitude Z for one navigation move.
     * <p>Absolute mode (navigation.absolute-altitude.enabled=true): target altitude = base-altitude + target cell layer * layerHeight,
     * independent of the current physical altitude. Horizontal and vertical moves both align to the absolute altitude of the target layer, so undershoot and drift are corrected on the next move,
     * every drone on a layer converges to the same altitude, and no drone ever crosses a layer boundary.
     * <p>Relative mode: keeps the original behaviour - a vertical move adds or subtracts layerHeight from the current altitude, a horizontal move keeps it.
     */
    private double computeTargetZ(MoveDirection direction, DroneStatus status, GridPoint nextPoint) {
        boolean vertical = (direction == MoveDirection.UP || direction == MoveDirection.DOWN);

        if (absoluteAltitudeEnabled) {
            int totalPerLayer = bigridCols * bigridRows;
            int targetLayer = (totalPerLayer > 0) ? nextPoint.gridIndex / totalPerLayer : 0;
            double targetZ = navigationBaseAltitude + targetLayer * layerHeight;
            System.out.println("  → " + (vertical ? "Vertical " + direction : "Horizontal")
                    + " (absolute): layer " + targetLayer + " -> "
                    + String.format(Locale.ROOT, "%.2f", targetZ) + "m (from "
                    + String.format(Locale.ROOT, "%.2f", status.z) + "m)");
            return targetZ;
        }

        // Relative mode (original behaviour)
        double targetZ;
        if (direction == MoveDirection.UP) {
            targetZ = status.z + layerHeight;
            System.out.println("  → Vertical UP: " + String.format("%.2f", status.z) + "m -> " + String.format("%.2f", targetZ) + "m");
        } else if (direction == MoveDirection.DOWN) {
            targetZ = status.z - layerHeight;
            System.out.println("  → Vertical DOWN: " + String.format("%.2f", status.z) + "m -> " + String.format("%.2f", targetZ) + "m");
        } else {
            targetZ = status.z;
            System.out.println("  → Horizontal move: keeping altitude " + String.format("%.2f", targetZ) + "m");
        }
        return targetZ;
    }

    /**
     * Extra safety check for diagonal movement:
     * for FORWARD_LEFT / FORWARD_RIGHT / BACK_LEFT / BACK_RIGHT,
     * both orthogonally adjacent cells on the swept path (for example forward and left) must be checked for occupancy or reservation.
     * If either cell is occupied or reserved by another drone, or is an obstacle, the diagonal move is treated as unsafe.
     */
    private boolean isDiagonalPathClear(String droneId, GridPoint current, GridPoint next, MoveDirection direction) {
        if (!(direction == MoveDirection.FORWARD_LEFT ||
              direction == MoveDirection.FORWARD_RIGHT ||
              direction == MoveDirection.BACK_LEFT ||
              direction == MoveDirection.BACK_RIGHT)) {
            return true;
        }
        
        int totalPerLayer = bigridCols * bigridRows;
        int layerIndex = current.gridIndex / totalPerLayer;
        int indexInLayer = current.gridIndex % totalPerLayer;
        int xIndex = indexInLayer / bigridRows;
        int yIndex = indexInLayer % bigridRows;
        
        List<Integer> cellsToCheck = new ArrayList<>(2);
        
        // Determine the two orthogonal cells to check, based on the diagonal direction
        switch (direction) {
            case FORWARD_LEFT -> {
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex + 1, yIndex);     // FORWARD
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex,     yIndex + 1); // LEFT
            }
            case FORWARD_RIGHT -> {
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex + 1, yIndex);     // FORWARD
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex,     yIndex - 1); // RIGHT
            }
            case BACK_LEFT -> {
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex - 1, yIndex);     // BACK
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex,     yIndex + 1); // LEFT
            }
            case BACK_RIGHT -> {
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex - 1, yIndex);     // BACK
                addIfValidDiagonalCheckCell(cellsToCheck, layerIndex, xIndex,     yIndex - 1); // RIGHT
            }
            default -> {
                return true;
            }
        }
        
        for (int idx : cellsToCheck) {
            // Ignore indices equal to the target cell (the target is checked separately by the bigraph rule and the reservation logic)
            if (idx == next.gridIndex) {
                continue;
            }
            
            // Dynamic collision check against the current positions of other drones only (obstacles and reservations are not considered here)
            for (Map.Entry<String, DronePosition> entry : dronePositions.entrySet()) {
                String otherId = entry.getKey();
                if (!otherId.equals(droneId)) {
                    DronePosition pos = entry.getValue();
                    if (pos.gridIndex == idx) {
                        DroneStatus otherStatus = droneStatuses.get(otherId);
                        GridPoint otherTarget = null;
                        if (otherStatus != null) {
                            if (otherStatus.isInEmergency && otherStatus.emergencyTarget != null) {
                                otherTarget = otherStatus.emergencyTarget;
                            } else {
                                otherTarget = droneTargets.get(otherId);
                            }
                        }
                        // Strategy 1: the other drone is heading for my current cell, so I am making way - allow the move
                        if (otherTarget != null && otherTarget.gridIndex == current.gridIndex) {
                            continue;
                        }
                        
                        // Strategy 2: deadlock detection - when both drones are waiting, the lower numeric ID goes first
                        DroneStatus myStatus = droneStatuses.get(droneId);
                        if (otherStatus != null && otherStatus.isWaiting
                                && myStatus != null && myStatus.isWaiting) {
                            int myIdx = Integer.parseInt(droneId.substring(1));
                            int otherIdx = Integer.parseInt(otherId.substring(1));
                            if (myIdx < otherIdx) {
                                System.out.println("  [Deadlock Break] " + droneId +
                                        " priority over waiting " + otherId + " at Grid[" + idx + "]");
                                continue;
                            }
                        }
                        
                        // Strategy 3: after waiting past the threshold, force the move through to avoid a permanent deadlock
                        if (myStatus != null && myStatus.consecutiveWaitCycles >= 5) {
                            System.out.println("  [Forced Pass] " + droneId +
                                    " forced through Grid[" + idx + "] after " +
                                    myStatus.consecutiveWaitCycles + " wait cycles");
                            myStatus.consecutiveWaitCycles = 0;
                            continue;
                        }
                        
                        System.out.println("  ⚠ " + droneId + " diagonal " + direction.name() +
                                " path blocked by another drone at Grid[" + idx + "], waiting/detour...");
                        return false;
                    }
                }
            }
        }
        
        return true;
    }
    
    /**
     * Helper: converts (layer, x, y) to a gridIndex and adds it to the check list, if it is in range.
     */
    private void addIfValidDiagonalCheckCell(List<Integer> cellsToCheck, int layerIndex, int xIndex, int yIndex) {
        if (xIndex >= 0 && xIndex < bigridCols && yIndex >= 0 && yIndex < bigridRows &&
            layerIndex >= 0 && layerIndex < bigridLayers) {
            int totalPerLayer = bigridCols * bigridRows;
            int idx = layerIndex * totalPerLayer + xIndex * bigridRows + yIndex;
            cellsToCheck.add(idx);
        }
    }
    
    /** Navigation control: dynamic A* + grid reservation + wait/detour */
    private void performNavigationControl() {
        if (!droneControlEnabled) {
            return;
        }
        
        long currentTime = System.currentTimeMillis();
        AbstractBigraphMatcher<PureBigraph> matcher = AbstractBigraphMatcher.create(PureBigraph.class);
        final long WAIT_TIMEOUT = 8000;  // Wait timeout in milliseconds
        
        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();
            
            // Must have taken off and had the take-off rule applied
            if (!status.hasTakenOff || !status.takeoffRuleApplied) {
                continue;
            }
            
            // If the target is reached, release every reservation and skip
            if (status.reachedDestination) {
                releaseAllGrids(droneId);
                continue;
            }
            
            try {
                // Before navigating, check whether an emergency response must be triggered
                // If the battery is already Low/Empty but no emergency response has been triggered, skip this navigation step
                // and wait for the status monitor thread to trigger the emergency response within the next second
                boolean needsEmergency = ("Low".equals(status.batteryLevel) || "Empty".equals(status.batteryLevel) 
                        || "Bad".equals(status.communicationStatus)) && !status.isInEmergency;
                if (needsEmergency) {
                    // Skip this navigation step and wait for the emergency response to be triggered
                    continue;
                }
                
                // Emergency response - pick the target: use emergencyTarget while in an emergency, otherwise the original target
                GridPoint target;
                if (status.isInEmergency && status.emergencyTarget != null) {
                    target = status.emergencyTarget;
                } else {
                    target = droneTargets.get(droneId);
                }
                
                if (target == null) {
                    System.err.println("  !! " + droneId + " no target configured");
                    continue;
                }
                
                // Get the current position
                DronePosition currentPos = dronePositions.get(droneId);
                if (currentPos == null || currentPos.gridIndex < 0) {
                    continue;
                }
                
                // Check whether a move is in progress (command sent but the ROS2 position has not updated yet)
                if (status.isMoving && status.movingToGrid != null) {
                    // Check whether the target grid has been reached
                    if (currentPos.gridIndex == status.movingToGrid) {
                        // Arrived - reset the movement state
                        System.out.println("  ✓ " + droneId + " arrived at Grid[" + status.movingToGrid + "]");
                        
                        // Release every stale reservation except the current position
                        releaseAllGrids(droneId);
                        
                        // Reserve the current position so no other drone can take it
                        tryReserveGrid(droneId, currentPos.gridIndex);
                        status.reservedGrid = currentPos.gridIndex;
                        
                        status.isMoving = false;
                        status.movingToGrid = null;
                        status.avoidGrids.clear();
                        status.consecutiveWaitCycles = 0;
                    } else {
                        // Move timeout: if movingToGrid is not reached for a long time (position drift or a desynchronised command), stop waiting and replan from the current position
                        if (currentTime - status.lastMoveTime > MOVE_TIMEOUT_MS) {
                            System.out.println("  ⏱ " + droneId + " move timeout: expected Grid[" + status.movingToGrid + "], current Grid[" + currentPos.gridIndex + "], re-planning from current position");
                            releaseGrid(droneId, status.movingToGrid);
                            status.reservedGrid = null;
                            status.isMoving = false;
                            status.movingToGrid = null;
                            // Do not continue; fall through and replan from currentPos
                        } else {
                            // Still moving - skip this navigation control step
                            continue;
                        }
                    }
                }
                
                // Check whether the final target has been reached
                if (currentPos.gridIndex == target.gridIndex) {
                    // Emergency response - handle the arrival according to the emergency type
                    if (status.isInEmergency) {
                        if ("EMPTY_BATTERY".equals(status.emergencyType) || "LOW_BATTERY_BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n🚨 [EMERGENCY LANDING] " + droneId + " reached emergency landing point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.emergencyLanded = true;
                            // No further movement after an emergency landing
                        } else if ("LOW_BATTERY".equals(status.emergencyType)) {
                            System.out.println("\n🔋 [RECHARGE POINT] " + droneId + " reached recharge point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;
                            // Wait for charging and take-off
                        } else if ("BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n📡 [HOME REACHED] " + droneId + " reached home Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;
                            // Wait for communication to recover
                        }
                    } else {
                        System.out.println("\n🎯 [Target Reached] " + droneId + " reached target Grid[" + target.gridIndex + "]");
                        status.reachedDestination = true;
                    }
                    
                    status.isMoving = false;
                    status.movingToGrid = null;
                    releaseAllGrids(droneId);
                    continue;
                }
                
                // Rate-limit the moves
                if (currentTime - status.lastMoveTime < 2000) { // one move every 2 seconds
                    continue;
                }
                
                GridPoint currentGridPoint = gridIndexToPoint(currentPos.gridIndex);
                
                long planStart = System.nanoTime();
                
                System.out.println("  [Path Planning] " + droneId + " current: Grid[" + currentPos.gridIndex + 
                        "](" + String.format("%.1f,%.1f,%.1f", currentGridPoint.x, currentGridPoint.y, currentGridPoint.z) + 
                        "), target: Grid[" + target.gridIndex + "](" + String.format("%.1f,%.1f", target.x, target.y) + ")");
                
                GridPoint nextPoint = planNextStep(droneId, currentGridPoint, target);
                
                if (nextPoint == null) {
                    logPlanningLatency(droneId, "navigation", planStart, System.nanoTime());
                    System.err.println("  !! " + droneId + " unable to plan path to target");
                    status.isWaiting = true;
                    continue;
                }
                
                int currentLayer = currentPos.gridIndex / (bigridCols * bigridRows);
                int nextLayer = nextPoint.gridIndex / (bigridCols * bigridRows);
                String layerInfo = (currentLayer != nextLayer) ? 
                        " [Layer " + currentLayer + " → Layer " + nextLayer + "]" : 
                        " [Layer " + currentLayer + "]";
                System.out.println("  [Next Step] Grid[" + nextPoint.gridIndex + 
                        "](" + String.format("%.1f,%.1f,%.1f", nextPoint.x, nextPoint.y, nextPoint.z) + ")" + layerInfo);
                
                MoveDirection direction = getMoveDirection(currentGridPoint, nextPoint);
                String directionName = direction.name();
                String navAction = directionName + " move";
                
                if (!isDiagonalPathClear(droneId, currentGridPoint, nextPoint, direction)) {
                    logPlanningLatency(droneId, navAction, planStart, System.nanoTime());
                    handleWaitingAndDetour(droneId, status, currentTime, currentGridPoint, target, nextPoint);
                    continue;
                }
                
                long planEnd = System.nanoTime();
                logPlanningLatency(droneId, navAction, planStart, planEnd);
                
                long matchStart = System.nanoTime();
                ParametricReactionRule<PureBigraph> moveRule = getMoveRuleForDirection(droneId, direction);
                Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(composite, moveRule).iterator();
                boolean ruleMatched = matchIterator.hasNext();
                long matchEnd = System.nanoTime();
                logMatchOnlyLatency(droneId, navAction, matchStart, matchEnd);
                
                if (!ruleMatched) {
                    System.out.println("  ⚠ " + droneId + " " + directionName + " rule not matched");
                    handleWaitingAndDetour(droneId, status, currentTime, currentGridPoint, target, nextPoint);
                    continue;
                }
                
                System.out.println("  ✓ " + droneId + " " + directionName + " rule matched");
                
                boolean reservationSuccess = tryReserveGrid(droneId, nextPoint.gridIndex);
                if (!reservationSuccess) {
                    System.out.println("  ⚠ " + droneId + " Grid[" + nextPoint.gridIndex + "] reserved by " + getGridReservation(nextPoint.gridIndex));
                    handleWaitingAndDetour(droneId, status, currentTime, currentGridPoint, target, nextPoint);
                    continue;
                }
                
                // Reservation succeeded
                if (status.isWaiting) {
                    long waitTime = currentTime - status.waitingStartTime;
                    System.out.println("  ✓ [Wait End] " + droneId + " successfully reserved Grid[" + nextPoint.gridIndex + 
                            "] (waited " + (waitTime / 1000) + "s)");
                    status.isWaiting = false;
                    status.waitingForGrid = null;
                    status.avoidGrids.clear();
                    status.consecutiveWaitCycles = 0;
                }
                status.reservedGrid = nextPoint.gridIndex;
                
                System.out.println(droneId + " locked Grid[" + nextPoint.gridIndex + "], preparing to move");
                
                int droneIndex = Integer.parseInt(droneId.substring(1));
                int port = droneControlStartPort + droneIndex;
                
                System.out.println("[Navigation] " + droneId + " from Grid[" + currentPos.gridIndex + 
                        "](" + String.format("%.1f,%.1f,%.1f", currentGridPoint.x, currentGridPoint.y, currentGridPoint.z) + ") " +
                        directionName + " moving to Grid[" + nextPoint.gridIndex + 
                        "](" + String.format("%.1f,%.1f,%.1f", nextPoint.x, nextPoint.y, nextPoint.z) + ")");
                
                // Compute the Z coordinate of the navigation target (absolute mode: the absolute altitude of the target layer, self-correcting; relative mode: current altitude +/- layerHeight)
                double targetZ = computeTargetZ(direction, status, nextPoint);
                
                if (navigateTo(droneId, port, nextPoint.x, nextPoint.y, targetZ)) {
                    status.lastMoveTime = currentTime;
                    
                    // Set the movement state (moving, waiting for the ROS2 position to update)
                    status.isMoving = true;
                    status.movingToGrid = nextPoint.gridIndex;
                    
                    // Release the reservation on the current position, if any
                    if (currentPos.gridIndex != nextPoint.gridIndex) {
                        releaseGrid(droneId, currentPos.gridIndex);
                    }
                } else {
                    System.err.println(" !! Navigation command failed");
                    // Release the reservation
                    releaseGrid(droneId, nextPoint.gridIndex);
                    status.reservedGrid = null;
                    status.isMoving = false;
                    status.movingToGrid = null;
                }
                
            } catch (ClassCastException e) {
                // PureBigraph.getPorts() assumes REFERENCE_PORT is an EList, but EMF/CDO sometimes returns an array, which makes the match fail
                System.err.println(" !! " + droneId + " nav: Bigraph model port structure incompatible (array vs list), skipping this cycle");
            } catch (Exception e) {
                System.err.println(" !! " + droneId + " nav error: " + e.getMessage());
            }
        }
    }
    
    /** Baseline navigation: A* + diagonal check + direct REST (no Bigraph match / grid reservation / wait) */
    private void performBaselineNavigationControl() {
        if (!droneControlEnabled) {
            return;
        }
        
        long currentTime = System.currentTimeMillis();
        
        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();
            
            if (!status.hasTakenOff || !status.takeoffRuleApplied) {
                continue;
            }
            
            if (status.reachedDestination) {
                continue;
            }
            
            try {
                boolean needsEmergency = ("Low".equals(status.batteryLevel) || "Empty".equals(status.batteryLevel) 
                        || "Bad".equals(status.communicationStatus)) && !status.isInEmergency;
                if (needsEmergency) {
                    continue;
                }
                
                GridPoint target;
                if (status.isInEmergency && status.emergencyTarget != null) {
                    target = status.emergencyTarget;
                } else {
                    target = droneTargets.get(droneId);
                }
                
                if (target == null) {
                    System.err.println("  !! " + droneId + " no target configured");
                    continue;
                }
                
                DronePosition currentPos = dronePositions.get(droneId);
                if (currentPos == null || currentPos.gridIndex < 0) {
                    continue;
                }
                
                if (status.isMoving && status.movingToGrid != null) {
                    if (currentPos.gridIndex == status.movingToGrid) {
                        System.out.println("  ✓ " + droneId + " arrived at Grid[" + status.movingToGrid + "]");
                        status.isMoving = false;
                        status.movingToGrid = null;
                        status.avoidGrids.clear();
                        status.consecutiveWaitCycles = 0;
                    } else if (currentTime - status.lastMoveTime > MOVE_TIMEOUT_MS) {
                        System.out.println("  ⏱ " + droneId + " move timeout: expected Grid[" + status.movingToGrid
                                + "], current Grid[" + currentPos.gridIndex + "], re-planning from current position");
                        status.isMoving = false;
                        status.movingToGrid = null;
                    } else {
                        continue;
                    }
                }
                
                if (currentPos.gridIndex == target.gridIndex) {
                    if (status.isInEmergency) {
                        if ("EMPTY_BATTERY".equals(status.emergencyType) || "LOW_BATTERY_BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n🚨 [EMERGENCY LANDING] " + droneId + " reached emergency landing point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.emergencyLanded = true;
                        } else if ("LOW_BATTERY".equals(status.emergencyType)) {
                            System.out.println("\n🔋 [RECHARGE POINT] " + droneId + " reached recharge point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;
                        } else if ("BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n📡 [HOME REACHED] " + droneId + " reached home Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;
                        }
                    } else {
                        System.out.println("\n🎯 [Target Reached] " + droneId + " reached target Grid[" + target.gridIndex + "]");
                        status.reachedDestination = true;
                    }
                    status.isMoving = false;
                    status.movingToGrid = null;
                    continue;
                }
                
                if (currentTime - status.lastMoveTime < MOVE_INTERVAL_MS) {
                    continue;
                }
                
                GridPoint currentGridPoint = gridIndexToPoint(currentPos.gridIndex);
                long planStart = System.nanoTime();
                
                System.out.println("  [Path Planning] " + droneId + " current: Grid[" + currentPos.gridIndex + 
                        "](" + String.format("%.1f,%.1f,%.1f", currentGridPoint.x, currentGridPoint.y, currentGridPoint.z) + 
                        "), target: Grid[" + target.gridIndex + "](" + String.format("%.1f,%.1f", target.x, target.y) + ")");
                
                GridPoint nextPoint = planNextStep(droneId, currentGridPoint, target);
                
                if (nextPoint == null) {
                    logPlanningLatency(droneId, "navigation", planStart, System.nanoTime());
                    System.err.println("  !! " + droneId + " unable to plan path to target");
                    continue;
                }
                
                int currentLayer = currentPos.gridIndex / (bigridCols * bigridRows);
                int nextLayer = nextPoint.gridIndex / (bigridCols * bigridRows);
                String layerInfo = (currentLayer != nextLayer) ? 
                        " [Layer " + currentLayer + " → Layer " + nextLayer + "]" : 
                        " [Layer " + currentLayer + "]";
                System.out.println("  [Next Step] Grid[" + nextPoint.gridIndex + 
                        "](" + String.format("%.1f,%.1f,%.1f", nextPoint.x, nextPoint.y, nextPoint.z) + ")" + layerInfo);
                
                MoveDirection direction = getMoveDirection(currentGridPoint, nextPoint);
                String directionName = direction.name();
                String navAction = directionName + " move";

                // Baseline: no protection at all - no diagonal sweep check (isDiagonalPathClear), no bigraph gate, no reservation.
                // The next A* step is executed unconditionally, so no deadlock from waiting; conflicts (two drones converging on one cell) really happen and are counted.
                logPlanningLatency(droneId, navAction, planStart, System.nanoTime());
                System.out.println("  → [Baseline] " + droneId + " executing " + directionName + " without Bigraph check");
                
                int droneIndex = Integer.parseInt(droneId.substring(1));
                int port = droneControlStartPort + droneIndex;
                
                System.out.println("[Navigation] " + droneId + " from Grid[" + currentPos.gridIndex + 
                        "](" + String.format("%.1f,%.1f,%.1f", currentGridPoint.x, currentGridPoint.y, currentGridPoint.z) + ") " +
                        directionName + " moving to Grid[" + nextPoint.gridIndex + 
                        "](" + String.format("%.1f,%.1f,%.1f", nextPoint.x, nextPoint.y, nextPoint.z) + ")");
                
                double targetZ = computeTargetZ(direction, status, nextPoint);
                
                if (navigateTo(droneId, port, nextPoint.x, nextPoint.y, targetZ)) {
                    status.lastMoveTime = currentTime;
                    status.isMoving = true;
                    status.movingToGrid = nextPoint.gridIndex;
                } else {
                    System.err.println(" !! Navigation command failed");
                    status.isMoving = false;
                    status.movingToGrid = null;
                }
                
            } catch (Exception e) {
                System.err.println(" !! " + droneId + " nav error: " + e.getMessage());
            }
        }
    }
    
    private void handleWaitingAndDetour(String droneId, DroneStatus status, long currentTime, 
            GridPoint currentPoint, GridPoint target, GridPoint blockedPoint) {
        if (!status.isWaiting) {
            status.isWaiting = true;
            status.waitingStartTime = currentTime;
            status.waitingForGrid = blockedPoint.gridIndex;
        }
        status.consecutiveWaitCycles++;
        
        if (currentTime - status.waitingStartTime > WAIT_TIMEOUT_MS) {
            Set<Integer> obstacles = getOccupiedGrids(droneId);
            obstacles.add(blockedPoint.gridIndex);
            List<GridPoint> detour = planPath(currentPoint, target, obstacles);
            
            if (detour.size() > 1 && detour.get(1).gridIndex != blockedPoint.gridIndex) {
                System.out.println("  [Detour] " + droneId + " \u2192 Grid[" + detour.get(1).gridIndex + "]");
                status.isWaiting = false;
                status.waitingForGrid = null;
                status.consecutiveWaitCycles = 0;
                // Record the blocked cell so the next planNextStep routes around it
                status.avoidGrids.add(blockedPoint.gridIndex);
                status.avoidGridsSetTime = currentTime;
            } else {
                // No detour found: still record the blocked cell, reset the timer and wait again
                status.avoidGrids.add(blockedPoint.gridIndex);
                status.avoidGridsSetTime = currentTime;
                status.waitingStartTime = currentTime;
            }
        }
    }
    
    /** Applies a bigraph rule and persists the result
     * @param droneId drone ID
     * @param rule the rule to apply
     * @param logPrefix log prefix
     * @return whether the rule was applied successfully
     */
    private boolean applyRuleAndPersist(String droneId, ParametricReactionRule<PureBigraph> rule, String logPrefix) {
        return applyRuleAndPersist(droneId, rule, logPrefix, logPrefix);
    }
    
    private boolean applyRuleAndPersist(String droneId, ParametricReactionRule<PureBigraph> rule, String logPrefix, String matchAction) {
        synchronized (cdoLock) {
            try {
                PureBigraph currentComposite = composite;
                AbstractBigraphMatcher<PureBigraph> matcher = AbstractBigraphMatcher.create(PureBigraph.class);
                long matchStart = System.nanoTime();
                Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(currentComposite, rule).iterator();
                
                if (matchIterator.hasNext()) {
                    long matchEnd = System.nanoTime();
                    logMatchOnlyLatency(droneId, matchAction, matchStart, matchEnd);
                    BigraphMatch<PureBigraph> match = matchIterator.next();
                    PureBigraph resultComposite = new PureReactiveSystem().buildParametricReaction(
                            currentComposite, match, rule);
                    
                    EObject insertedComposite = template.insert(resultComposite.getInstanceModel(), "/composed");
                    composite = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedComposite, sig());
                    cdoIdComposed = CDOUtil.getCDOObject(composite.getInstanceModel()).cdoID();
                    
                    System.out.println("  ✓ Rule applied, " + droneId + " " + logPrefix);
                    System.out.println("  ✓ Updated model persisted (CDOID: " + cdoIdComposed + ")");
                    return true;
                } else {
                    System.out.println("  ○ Rule not matched, " + droneId + " may already be in target status");
                    return false;
                }
            } catch (ClassCastException e) {
                // PureBigraph.getPorts() assumes REFERENCE_PORT is an EList, but EMF/CDO sometimes returns an array, causing [Ljava.lang.Object; cannot be cast to List
                System.err.println("  !! Bigraph model port structure incompatible (array vs list), skipping rule: " + droneId + " " + logPrefix);
                return false;
            } catch (Exception e) {
                System.err.println("  !! CDO operation failed: " + e.getMessage());
                e.printStackTrace();
                return false;
            }
        }
    }
    
    /**
     * Checks and applies the take-off rule once the drone has physically taken off
     * Note: this method modifies the composite field and cdoIdComposed
     */
    private boolean checkAndApplyTakeoffRules() {
        boolean anyRuleApplied = false;
        
        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();
            
            // The drone has taken off but the rule has not been applied yet
            if (status.hasTakenOff && !status.takeoffRuleApplied) {
                if (baselineMode) {
                    System.out.println("\n [Baseline] " + droneId + " took off (altitude: " +
                            String.format("%.3f", status.z) + "m), marking flying (no Bigraph rule apply)");
                    status.status = "flying";
                    status.takeoffRuleApplied = true;
                    anyRuleApplied = true;
                    continue;
                }
                try {
                    System.out.println("\n [Rule Apply] " + droneId + " took off (altitude: " + 
                            String.format("%.3f", status.z) + "m), applying takeoff rule...");
                    
                    ParametricReactionRule<PureBigraph> takeoffRule = droneTakeOffRule(droneId);
                    if (applyRuleAndPersist(droneId, takeoffRule, "status updated to flying", "apply takeoff rule")) {
                        status.status = "flying"; // Keep the new state in memory; it is used when the world model is rebuilt
                        status.takeoffRuleApplied = true;
                        anyRuleApplied = true;
                    } else {
                        // When the take-off rule does not match, log the reason but still update the in-memory state
                        // because the drone has physically taken off and the states must stay consistent
                        String reason = "";
                        if (!"Normal".equals(status.batteryLevel)) {
                            reason = "Battery is " + status.batteryLevel + " (rule requires Normal)";
                        } else if (!"Normal".equals(status.communicationStatus)) {
                            reason = "Communication is " + status.communicationStatus + " (rule requires Normal)";
                        } else {
                            reason = "Bigraph model may not be updated yet";
                        }
                        System.out.println("  ⚠ Takeoff rule not matched: " + reason);
                        System.out.println("  → Physical state is flying, updating memory status...");
                        
                        // Update the in-memory state so droneModelFromRosPositions sets the bigraph state correctly on the next update
                        status.status = "flying";//? todo: test if this is needed
                        status.takeoffRuleApplied = true;
                    }
                } catch (Exception e) {
                    System.err.println("  !! Failed to apply takeoff rule: " + e.getMessage());
                    e.printStackTrace();
                }
            }
        }
        
        return anyRuleApplied;
    }
    
    /**
     * Checks and applies the landing rule once the drone has reached its target and physically landed
     * Note: this method modifies the composite field and cdoIdComposed
     */
    private boolean checkAndApplyLandingRules() {
        boolean anyRuleApplied = false;
        long currentTime = System.currentTimeMillis();
        
        // Handle the landing logic of each drone independently
        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();
            
            // Step 1: if the target has been reached, send the landing command immediately (without waiting for the other drones)
            if (status.reachedDestination && !status.landingCommandSent) {
                try {
                    System.out.println("\n[Target Reached] " + droneId + " reached target, beginning landing immediately...");
                    
                    int droneIndex = Integer.parseInt(droneId.substring(1));
                    int port = droneControlStartPort + droneIndex;
                    
                    if (beginLanding(droneId, port)) {
                        status.landingCommandSent = true;
                        status.landingCommandTime = currentTime;
                        System.out.println("  ✓ " + droneId + " landing command sent");
                    } else {
                        System.err.println(" !! Unable to send landing command");
                    }
                } catch (Exception e) {
                    System.err.println(" !! Error sending landing command: " + e.getMessage());
                    e.printStackTrace();
                }
                continue; // After sending the landing command, move on to the next drone
            }
            
            // Step 2: if the drone has landed, apply the landing rule (handled independently)
            if (status.landingCommandSent && !status.landingRuleApplied) {
                long timeSinceLandingCommand = currentTime - status.landingCommandTime;
                
                if (timeSinceLandingCommand >= 2000 && status.hasLanded) { // after 2 seconds and once landed
                    try {
                        System.out.println("\n [Rule Apply] " + droneId + " landed (altitude: " + 
                                String.format("%.3f", status.z) + "m), applying landing rule...");
                        
                        ParametricReactionRule<PureBigraph> landingRule = droneLandingRule(droneId);
                        if (applyRuleAndPersist(droneId, landingRule, "status updated to Landed", "apply landing rule")) {
                            status.status = "Landed";
                            status.landingRuleApplied = true;
                            anyRuleApplied = true;
                        } else {
                            status.landingRuleApplied = true;
                        }
                    } catch (Exception e) {
                        System.err.println("  !! Failed to apply landing rule: " + e.getMessage());
                        e.printStackTrace();
                    }
                }
            }
        }
        
        return anyRuleApplied;
    }

    /**
     * Continuously monitors the battery and communication state and applies the matching rules
     * Runs on its own thread, in parallel with position updates and navigation control
     */
    // ==================== Fault Injection ====================
    // A small built-in HTTP service (from the JDK, no extra dependency) allows faults to be injected at runtime.
    // What is injected are synthetic sensor readings; everything downstream (bigraph rules, emergency response) follows the real code path.
    //
    // Usage examples (curl from another terminal, or open the URL in a browser):
    //   Set D1 battery to Low:        curl "http://localhost:8090/inject/battery?drone=D1&level=Low"
    //   Set D1 battery to Empty:      curl "http://localhost:8090/inject/battery?drone=D1&level=Empty"
    //   Set D2 communication to Bad:  curl "http://localhost:8090/inject/comm?drone=D2&state=Bad"
    //   Set a raw voltage:            curl "http://localhost:8090/inject/battery?drone=D1&value=3.12"
    //   Clear the injection (restore real values): curl "http://localhost:8090/inject/battery?drone=D1&level=clear"
    //                         curl "http://localhost:8090/inject/comm?drone=D2&state=clear"
    //   Show the current injection state:          curl "http://localhost:8090/inject/status"
    private void startFaultInjectionServer() {
        if (!faultInjectionEnabled) {
            System.out.println("Fault-injection HTTP server disabled (fault.injection.enabled=false)");
            return;
        }
        try {
            HttpServer server = HttpServer.create(new InetSocketAddress(faultInjectionPort), 0);
            server.createContext("/inject/battery", ex -> handleInject(ex, true));
            server.createContext("/inject/comm", ex -> handleInject(ex, false));
            server.createContext("/inject/status", this::handleInjectStatus);
            server.setExecutor(null); // use the default executor
            server.start();
            System.out.println("\n========================================");
            System.out.println("✓ Fault-injection HTTP server listening on port " + faultInjectionPort);
            System.out.println("  battery: curl \"http://localhost:" + faultInjectionPort + "/inject/battery?drone=D1&level=Low|Empty|Normal|clear\"");
            System.out.println("  comm:    curl \"http://localhost:" + faultInjectionPort + "/inject/comm?drone=D2&state=Bad|Normal|clear\"");
            System.out.println("  status:  curl \"http://localhost:" + faultInjectionPort + "/inject/status\"");
            System.out.println("========================================\n");
        } catch (Exception e) {
            System.err.println("!! Failed to start fault-injection server on port " + faultInjectionPort + ": " + e.getMessage());
        }
    }

    /** Handles /inject/battery and /inject/comm; battery=true means a battery injection, otherwise a communication injection */
    private void handleInject(HttpExchange ex, boolean battery) throws IOException {
        Map<String, String> q = parseQuery(ex.getRequestURI().getRawQuery());
        String drone = q.get("drone");
        String level = battery ? firstNonNull(q.get("level"), q.get("value"))
                               : firstNonNull(q.get("state"), q.get("value"));

        if (drone == null || level == null) {
            respond(ex, 400, "missing parameter. need drone= and level=/state=/value=\n");
            return;
        }
        DroneStatus st = droneStatuses.get(drone);
        if (st == null) {
            respond(ex, 404, "unknown drone '" + drone + "'. available: " + droneStatuses.keySet() + "\n");
            return;
        }

        String msg = battery ? applyBatteryInjection(st, level) : applyCommInjection(st, level);
        if (msg == null) {
            respond(ex, 400, "invalid value '" + level + "'\n");
            return;
        }
        System.out.println("🧪 [INJECT] " + drone + " " + msg);
        respond(ex, 200, "OK: " + drone + " " + msg + "\n");
    }

    /** Sets a battery injection. Returns a description string, or null for an invalid value. */
    private String applyBatteryInjection(DroneStatus st, String level) {
        Double v;
        switch (level.toLowerCase(Locale.ROOT)) {
            case "normal": v = INJECT_BATTERY_NORMAL_V; break;
            case "low":    v = INJECT_BATTERY_LOW_V;    break;
            case "empty":  v = INJECT_BATTERY_EMPTY_V;  break;
            case "clear": case "off": case "none":
                st.injectedBatteryVoltage = null;
                return "battery injection cleared (back to real ROS value)";
            default:
                try { v = Double.parseDouble(level); } catch (NumberFormatException e) { return null; }
        }
        st.injectedBatteryVoltage = v;
        st.batteryVoltage = v; // takes effect immediately, without waiting for the next ROS message
        return "battery injected -> " + String.format(Locale.ROOT, "%.2fV", v);
    }

    /** Sets a communication injection. Returns a description string, or null for an invalid value. */
    private String applyCommInjection(DroneStatus st, String state) {
        Integer r;
        switch (state.toLowerCase(Locale.ROOT)) {
            case "normal": r = INJECT_RSSI_NORMAL; break;
            case "bad":    r = INJECT_RSSI_BAD;    break;
            case "clear": case "off": case "none":
                st.injectedRssi = null;
                return "comm injection cleared (back to real ROS value)";
            default:
                try { r = Integer.parseInt(state); } catch (NumberFormatException e) { return null; }
        }
        st.injectedRssi = r;
        st.rssi = r; // takes effect immediately
        return "comm injected -> RSSI " + r;
    }

    /** Handles /inject/status: returns the current injection and state of every drone */
    private void handleInjectStatus(HttpExchange ex) throws IOException {
        StringBuilder sb = new StringBuilder();
        for (Map.Entry<String, DroneStatus> e : droneStatuses.entrySet()) {
            DroneStatus s = e.getValue();
            sb.append(e.getKey())
              .append(": battery=").append(s.batteryLevel)
              .append(String.format(Locale.ROOT, "(%.2fV)", s.batteryVoltage))
              .append(s.injectedBatteryVoltage != null ? "[INJECTED]" : "")
              .append(", comm=").append(s.communicationStatus)
              .append("(RSSI ").append(s.rssi).append(")")
              .append(s.injectedRssi != null ? "[INJECTED]" : "")
              .append("\n");
        }
        respond(ex, 200, sb.toString());
    }

    private static Map<String, String> parseQuery(String raw) {
        Map<String, String> m = new HashMap<>();
        if (raw == null || raw.isEmpty()) return m;
        for (String pair : raw.split("&")) {
            int i = pair.indexOf('=');
            if (i > 0) m.put(pair.substring(0, i), pair.substring(i + 1));
        }
        return m;
    }

    private static String firstNonNull(String a, String b) { return a != null ? a : b; }

    private static void respond(HttpExchange ex, int code, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", "text/plain; charset=utf-8");
        ex.sendResponseHeaders(code, bytes.length);
        try (OutputStream os = ex.getResponseBody()) {
            os.write(bytes);
        }
    }

    private void continuousStatusMonitoring() {
        try {
            System.out.println("\n========================================");
            System.out.println("Battery and communication status monitoring thread started");
            System.out.println("========================================\n");
            
            while (true) {
                TimeUnit.MILLISECONDS.sleep(1000);  // Check once per second
                
                if (!rosUpdateEnabled) {
                    continue;
                }
                
                // Check the state of each drone and apply the matching rules
                for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
                    String droneId = entry.getKey();
                    DroneStatus status = entry.getValue();
                    
                    try {
                        // Check the battery state
                        checkAndApplyBatteryRule(droneId, status);
                        
                        // Check the communication state
                        checkAndApplyCommunicationRule(droneId, status);
                        
                        // Checks for and triggers an emergency response
                        checkAndTriggerEmergencyResponse(droneId, status);

                        // Check for recovery from the emergency state (communication restored)
                        checkEmergencyRecovery(droneId, status);
                    } catch (Exception e) {
                        System.err.println("  !! Error processing " + droneId + " status monitoring: " + e.getMessage());
                        e.printStackTrace();
                    }
                }
            }
        } catch (Exception e) {
            System.err.println("!! Status monitoring thread exception: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /** Checks and applies the battery state-transition rules */
    private void checkAndApplyBatteryRule(String droneId, DroneStatus status) {
        long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;
        boolean inTakeoffProtection = (timeSinceTakeoff < TAKEOFF_PROTECTION_MS) && 
                status.takeoffCommandSent && !status.takeoffRuleApplied;
        
        // Apply transitions repeatedly until the battery state matches the current voltage.
        // Important: when Empty is injected (e.g. 3.0 V), Normal -> Low -> Empty completes within a single monitoring cycle,
        // so the following emergency check sees Empty and triggers a landing in place, instead of stopping at Low and triggering a return to base.
        boolean changed = true;
        while (changed) {
            changed = false;
            if (status.batteryVoltage < BATTERY_EMPTY_THRESHOLD && "Low".equals(status.batteryLevel)) {
                applyBatteryTransition(droneId, status, "Low", "Empty", true);
                changed = true;
            } else if (status.batteryVoltage < BATTERY_LOW_THRESHOLD && "Normal".equals(status.batteryLevel) && !inTakeoffProtection) {
                applyBatteryTransition(droneId, status, "Normal", "Low", true);
                changed = true;
            } else if (status.batteryVoltage >= BATTERY_LOW_THRESHOLD && "Low".equals(status.batteryLevel)) {
                if (!applyBatteryTransition(droneId, status, "Low", "Normal", false)) {
                    status.bigraphBatteryNeedsSync = true;
                } else {
                    status.bigraphBatteryNeedsSync = false;
                }
                changed = true;
            } else if (status.batteryVoltage >= BATTERY_EMPTY_THRESHOLD && status.batteryVoltage < BATTERY_LOW_THRESHOLD && "Empty".equals(status.batteryLevel)) {
                applyBatteryTransition(droneId, status, "Empty", "Low", false);
                changed = true;
            }
        }
    }
    
    private boolean applyBatteryTransition(String droneId, DroneStatus status, String from, String to, boolean setApplied) {
        try {
            System.out.println("[Battery] " + droneId + ": " + from + " → " + to + " (" + String.format("%.2fV", status.batteryVoltage) + ")");
            boolean success = applyRuleAndPersist(droneId, createBatteryRule(droneId, from, to), "battery → " + to,
                    "battery " + from + "→" + to);
            status.batteryLevel = to;
            status.batteryRuleApplied = setApplied;
            return success;
        } catch (Exception e) {
            System.err.println("  !! Battery rule failed: " + e.getMessage());
            status.batteryLevel = to;
            return false;
        }
    }
    
    /** Checks and applies the communication state-transition rules */
    private void checkAndApplyCommunicationRule(String droneId, DroneStatus status) {
        try {
            if (status.rssi >= RSSI_BAD_THRESHOLD && !"Bad".equals(status.communicationStatus)) {
                applyCommunicationTransition(droneId, status, "Normal", "Bad");
            } else if (status.rssi < RSSI_BAD_THRESHOLD && "Bad".equals(status.communicationStatus)) {
                applyCommunicationTransition(droneId, status, "Bad", "Normal");
            }
        } catch (Exception e) {
            System.err.println("  !! Communication rule failed: " + e.getMessage());
        }
    }
    
    private void applyCommunicationTransition(String droneId, DroneStatus status, String from, String to) throws Exception {
        System.out.println("[Comm] " + droneId + ": " + from + " → " + to + " (RSSI: " + status.rssi + ")");
        applyRuleAndPersist(droneId, createCommunicationRule(droneId, from, to), "comm → " + to, "comm " + from + "→" + to);
        status.communicationStatus = to;
        status.communicationRuleApplied = true;
    }
    
    /** Checks for and triggers an emergency response */
    private void checkAndTriggerEmergencyResponse(String droneId, DroneStatus status) {
        if (!status.hasTakenOff) return;

        boolean lowBattery = "Low".equals(status.batteryLevel);
        boolean emptyBattery = "Empty".equals(status.batteryLevel);
        boolean badComm = "Bad".equals(status.communicationStatus);

        // Already in an emergency: only an escalation to a higher priority is allowed - an empty battery must land in place,
        // overriding an in-progress Low or BAD_COMM return to base (the paper's "landing in place takes priority over a long return flight").
        if (status.isInEmergency) {
            boolean alreadyLanding = "EMPTY_BATTERY".equals(status.emergencyType)
                    || "LOW_BATTERY_BAD_COMM".equals(status.emergencyType);
            if (emptyBattery && !alreadyLanding && !status.emergencyLanded) {
                System.out.println("\n🚨 [ESCALATE] " + droneId + " battery Empty during "
                        + status.emergencyType + " → switching to immediate local landing");
                triggerEmergencyLanding(droneId, status, "EMPTY_BATTERY");
            }
            return;
        }

        long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;
        boolean inTakeoffProtection = (timeSinceTakeoff < TAKEOFF_PROTECTION_MS) &&
                status.takeoffCommandSent && !status.takeoffRuleApplied;

        if (emptyBattery) {
            triggerEmergencyLanding(droneId, status, "EMPTY_BATTERY");
        } else if (lowBattery && badComm) {
            triggerEmergencyLanding(droneId, status, "LOW_BATTERY_BAD_COMM");
        } else if (lowBattery && !badComm && !inTakeoffProtection) {
            triggerReturnForRecharge(droneId, status);
        } else if (badComm && !lowBattery && !emptyBattery && !inTakeoffProtection) {
            triggerReturnToHome(droneId, status);
        }
    }
    
    /** Triggers an emergency landing */
    private void triggerEmergencyLanding(String droneId, DroneStatus status, String emergencyType) {
        System.out.println("\n🚨 [EMERGENCY] " + droneId + " " + emergencyType);
        
        DronePosition currentPos = dronePositions.get(droneId);
        if (currentPos == null) return;
        
        int emergencyGridIndex = coordinateToGridIndex(currentPos.x, currentPos.y, gridOriginZ);
        if (emergencyGridIndex < 0) return;
        
        GridPoint emergencyLandingPoint = gridIndexToPoint(emergencyGridIndex);
        if (emergencyLandingPoint == null) return;
        
        setEmergencyState(droneId, status, emergencyType, emergencyLandingPoint);
        System.out.println("  → Emergency target: " + emergencyLandingPoint);
    }
    
    /** Triggers a return to a charging station */
    private void triggerReturnForRecharge(String droneId, DroneStatus status) {
        System.out.println("\n⚠️ [LOW_BATTERY] " + droneId + " (" + String.format("%.2fV", status.batteryVoltage) + ")");
        
        DronePosition currentPos = dronePositions.get(droneId);
        if (currentPos == null) return;
        
        GridPoint currentPoint = gridIndexToPoint(currentPos.gridIndex);
        if (currentPoint == null) return;
        
        GridPoint startPoint = (status.startGridIndex != null) ? gridIndexToPoint(status.startGridIndex) : null;
        
        setEmergencyState(droneId, status, "LOW_BATTERY", null);
        
        // Find nearest available charging station
        GridPoint selectedStation = findNearestAvailableChargingStation(droneId, currentPoint);
        double distToHome = (startPoint != null) ? heuristic(currentPoint, startPoint) : Double.MAX_VALUE;
        
        if (selectedStation != null && heuristic(currentPoint, selectedStation) <= distToHome) {
            // A charging station exists and is closer - head for the charging station
            status.emergencyTarget = selectedStation;
            System.out.println("  → Charging Station: " + selectedStation);
        } else if (startPoint != null) {
            // No usable charging station, or it is further away - return to the start point
            status.emergencyTarget = startPoint;
            if (selectedStation == null) {
                String reason = chargingStations.isEmpty() 
                        ? "No charging station configured" 
                        : "Charging station(s) occupied/reserved/targeted by others";
                System.out.println("  → " + reason + ", returning to start point: " + startPoint);
            } else {
                System.out.println("  → Start point is closer, returning to: " + startPoint);
            }
        } else {
            // The start point is missing too (an abnormal case) - perform an emergency landing
            System.err.println("  !! No start point available, triggering emergency landing");
            triggerEmergencyLanding(droneId, status, "LOW_BATTERY");
            return;
        }
    }
    
    /** Triggers a return to the start point */
    private void triggerReturnToHome(String droneId, DroneStatus status) {
        System.out.println("\n📡 [BAD_COMM] " + droneId + " (RSSI: " + status.rssi + ")");
        
        GridPoint startPoint = (status.startGridIndex != null) ? gridIndexToPoint(status.startGridIndex) : null;
        if (startPoint == null) return;
        
        setEmergencyState(droneId, status, "BAD_COMM", startPoint);
        System.out.println("  → Start Point: " + startPoint);
    }
    
    private void setEmergencyState(String droneId, DroneStatus status, String type, GridPoint target) {
        status.isInEmergency = true;
        status.emergencyType = type;
        status.isMoving = false;
        status.movingToGrid = null;
        status.isWaiting = false;
        status.waitingForGrid = null;
        releaseAllGrids(droneId);
        if (status.originalTarget == null) status.originalTarget = droneTargets.get(droneId);
        if (target != null) status.emergencyTarget = target;
    }
    
    private GridPoint findNearestAvailableChargingStation(String droneId, GridPoint currentPoint) {
        GridPoint nearest = null;
        double minDist = Double.MAX_VALUE;
        
        for (GridPoint station : chargingStations) {
            boolean occupied = dronePositions.entrySet().stream()
                    .anyMatch(e -> !e.getKey().equals(droneId) && e.getValue().gridIndex == station.gridIndex);
            String reservedBy = getGridReservation(station.gridIndex);
            boolean reserved = (reservedBy != null && !reservedBy.equals(droneId));
            boolean targeted = droneStatuses.entrySet().stream()
                    .anyMatch(e -> !e.getKey().equals(droneId) && e.getValue().isInEmergency && 
                            "LOW_BATTERY".equals(e.getValue().emergencyType) &&
                            e.getValue().emergencyTarget != null &&
                            e.getValue().emergencyTarget.gridIndex == station.gridIndex);
            
            if (!occupied && !reserved && !targeted) {
                double dist = heuristic(currentPoint, station);
                if (dist < minDist) {
                    minDist = dist;
                    nearest = station;
                }
            }
        }
        return nearest;
    }
    
    
    /** Checks for recovery from the emergency state (logs communication recovery) */
    private void checkEmergencyRecovery(String droneId, DroneStatus status) {
        if ("BAD_COMM".equals(status.emergencyType) && "Normal".equals(status.communicationStatus)) {
            System.out.println("📡 [INFO] " + droneId + " Comm recovered, continuing home");
        }
    }
    
    // ========================================
    // ROS2 subscriptions and position management
    // ========================================
    
    /**
     * Subscribes to a ROS2 topic to obtain drone positions
     * @param host ROS bridge host address
     * @param topic topic name
     * @param type message type
     * @param handler message handler
     */
    // Shared rosbridge connection: every topic subscription reuses one WebSocket, instead of opening a connection per topic,
    // which otherwise causes "Could not create WebSocket: Connection failed" with many drones or topics (especially in sim mode).
    private Ros sharedRos;

    /** Ensures the shared rosbridge connection is up; returns true on success. */
    private synchronized boolean ensureRosConnected(String host) {
        if (sharedRos != null) {
            return true;
        }
        try {
            Ros ros = new Ros(host);
            ros.connect();
            sharedRos = ros;
            System.out.println("✓ Connected to rosbridge at " + host + ":9090 (shared connection)");
            return true;
        } catch (Exception e) {
            System.err.println("!! rosbridge connect error at " + host + ":9090 "
                    + "(is rosbridge_server / the sim position publisher running?): " + e.getMessage());
            return false;
        }
    }

    private void subscribeRosTopic(String host, String topic, String type, RosMessageHandler handler) {
        try {
            if (!ensureRosConnected(host)) {
                System.err.println("Failed to subscribe (no rosbridge connection) to topic: " + topic);
                return;
            }
            Topic rosTopic = new Topic(sharedRos, topic, type);
            rosTopic.subscribe(message -> {
                try {
                    handler.handle(message);
                } catch (Exception e) {
                    System.err.println("Error processing ROS2 message (topic: " + topic + "): " + e.getMessage());
                    e.printStackTrace();
                }
            });
            System.out.println("✓ Subscribed to ROS2 topic: " + topic);
        } catch (Exception e) {
            System.err.println("Failed to subscribe to ROS2 topic (topic: " + topic + "): " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Initialises the ROS2 subscriptions for every drone
     */
    private void initializeRosSubscriptions() {
        if (!rosUpdateEnabled) {
            System.out.println("\n========================================");
            System.out.println("ROS2 update function not enabled");
            System.out.println("To enable, set: ros.update.enabled=true");
            System.out.println("========================================\n");
            return;
        }
        
        System.out.println("\n========================================");
        System.out.println("Initializing ROS2 subscriptions");
        System.out.println("========================================");
        System.out.println("ROS Bridge Host: " + rosBridgeHost);
        System.out.println("Drone Count: " + configuredDroneCount);
        System.out.println("ROS Sim Mode: " + rosSimMode);
        
        // Initialise the position and status of every drone to defaults
        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            // Initialise to the default position (layer 1, bottom-left corner, grid index i)
            dronePositions.put(droneId, new DronePosition(droneId, 0, 0, 0, i));
            // Initialise the status
            droneStatuses.put(droneId, new DroneStatus(droneId));
        }
        System.out.println("✓ Initialized default positions and status for " + configuredDroneCount + " drones");
        
        // Position subscription: one topic in simulation mode, one topic per drone otherwise
        if (rosSimMode) {
            // Simulation mode: read from /cf_positions_path (nav_msgs/Path), identifying cf231/cf232/... via poses[].header.frame_id
            subscribeRosTopic(rosBridgeHost, "/cf_positions_path", "nav_msgs/Path", message -> {
                try {
                    JsonObject jsonObject = message.toJsonObject();
                    if (!jsonObject.containsKey("poses")) {
                        return;
                    }
                    javax.json.JsonArray poses = jsonObject.getJsonArray("poses");
                    if (poses == null || poses.isEmpty()) {
                        return;
                    }
                    for (int i = 0; i < poses.size(); i++) {
                        JsonObject poseStamped = poses.getJsonObject(i);
                        JsonObject header = poseStamped.getJsonObject("header");
                        if (header == null || !header.containsKey("frame_id")) {
                            continue;
                        }
                        String frameId = header.getString("frame_id");
                        // frame_id such as "cf231", "cf232", "cf233" -> 231,232,233 -> D0, D1, D2
                        int cfNumber;
                        try {
                            if (!frameId.startsWith("cf")) {
                                continue;
                            }
                            cfNumber = Integer.parseInt(frameId.substring(2));
                        } catch (NumberFormatException e) {
                            continue;
                        }
                        int droneIndex = cfNumber - 231;
                        if (droneIndex < 0 || droneIndex >= configuredDroneCount) {
                            continue;
                        }
                        String droneId = "D" + droneIndex;
                        JsonObject pose = poseStamped.getJsonObject("pose");
                        if (pose == null || !pose.containsKey("position")) {
                            continue;
                        }
                        JsonObject position = pose.getJsonObject("position");
                        double x = position.getJsonNumber("x").doubleValue();
                        double y = position.getJsonNumber("y").doubleValue();
                        double z = position.getJsonNumber("z").doubleValue();
                        recordRosPoseSample();
                        int gridIndex = coordinateToGridIndex(x, y, z);
                        DroneStatus status = droneStatuses.get(droneId);
                        if (status != null) {
                            status.z = z;
                            final long TAKEOFF_COOLDOWN = 5000;
                            long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;
                            if (z > 0.1 && !status.hasTakenOff) {
                                status.hasTakenOff = true;
                                status.takeoffTime = System.currentTimeMillis();
                                System.out.println("[Takeoff Detection] " + droneId + " took off! Altitude: " + String.format("%.3f", z) + "m");
                            }
                            if (z <= 0.1 && status.landingCommandSent && !status.hasLanded && timeSinceTakeoff > TAKEOFF_COOLDOWN) {
                                status.hasLanded = true;
                                System.out.println("[Landing Detection] " + droneId + " landed! Altitude: " + String.format("%.3f", z) + "m");
                            }
                        }
                        DronePosition oldPos = dronePositions.get(droneId);
                        DronePosition newPos = new DronePosition(droneId, x, y, z, gridIndex);
                        boolean shouldUpdate = (oldPos == null || oldPos.gridIndex != gridIndex);
                        if (shouldUpdate) {
                            dronePositions.put(droneId, newPos);
                            checkCollisionRisk();
                            System.out.println("[ROS2] " + droneId + " (" + frameId + " sim): " +
                                    String.format("(%.2f, %.2f, %.2f)", x, y, z) + " -> Grid[" + gridIndex + "]" +
                                    (oldPos != null ? " (From Grid[" + oldPos.gridIndex + "])" : " (Initial Position)"));
                        }
                    }
                } catch (Exception e) {
                    System.err.println("  ⚠ Error parsing /cf_positions_path message: " + e.getMessage());
                }
            });
        } else {
            // Real mode: each drone subscribes to its own /cfXXX/pose
            for (int i = 0; i < configuredDroneCount; i++) {
                int droneIndex = i;
                String droneId = "D" + i;
                int cfNumber = 231 + i;
                String topic = "/cf" + cfNumber + "/pose";
                
                subscribeRosTopic(rosBridgeHost, topic, "geometry_msgs/PoseStamped", message -> {
                    JsonObject jsonObject = message.toJsonObject();
                    
                    JsonObject pose = jsonObject.getJsonObject("pose");
                    JsonObject position = pose.getJsonObject("position");
                    double x = position.getJsonNumber("x").doubleValue();
                    double y = position.getJsonNumber("y").doubleValue();
                    double z = position.getJsonNumber("z").doubleValue();
                    
                    recordRosPoseSample();
                    int gridIndex = coordinateToGridIndex(x, y, z);
                    
                    DroneStatus status = droneStatuses.get(droneId);
                    if (status != null) {
                        status.z = z;
                        
                        final long TAKEOFF_COOLDOWN = 5000;
                        long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;
                        
                        if (z > 0.1 && !status.hasTakenOff) {
                            status.hasTakenOff = true;
                            status.takeoffTime = System.currentTimeMillis();
                            System.out.println("[Takeoff Detection] " + droneId + " took off! Altitude: " + String.format("%.3f", z) + "m");
                        }
                        
                        if (z <= 0.1 && status.landingCommandSent && !status.hasLanded && timeSinceTakeoff > TAKEOFF_COOLDOWN) {
                            status.hasLanded = true;
                            System.out.println("[Landing Detection] " + droneId + " landed! Altitude: " + String.format("%.3f", z) + "m");
                        }
                    }
                    
                    DronePosition oldPos = dronePositions.get(droneId);
                    DronePosition newPos = new DronePosition(droneId, x, y, z, gridIndex);
                    
                    boolean shouldUpdate = false;
                    if (oldPos == null || oldPos.gridIndex != gridIndex) {
                        shouldUpdate = true;
                    }
                    
                    if (shouldUpdate) {
                        dronePositions.put(droneId, newPos);
                        checkCollisionRisk();
                        System.out.println("[ROS2] " + droneId + " (cf" + cfNumber + "): " +
                                String.format("(%.2f, %.2f, %.2f)", x, y, z) + " -> Grid[" + gridIndex + "]" +
                                (oldPos != null ? " (From Grid[" + oldPos.gridIndex + "])" : " (Initial Position)"));
                    }
                });
            }
        }
        
        // The status topic (battery voltage and RSSI) is always subscribed per drone
        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            int cfNumber = 231 + i;
            String statusTopic = "/cf" + cfNumber + "/status";
            subscribeRosTopic(rosBridgeHost, statusTopic, "crazyflie_interfaces/msg/Status", message -> {
                try {
                    JsonObject jsonObject = message.toJsonObject();
                    
                    if (jsonObject.containsKey("battery_voltage") && jsonObject.containsKey("rssi")) {
                        double batteryVoltage = jsonObject.getJsonNumber("battery_voltage").doubleValue();
                        int rssi = jsonObject.getJsonNumber("rssi").intValue();
                        
                        DroneStatus status = droneStatuses.get(droneId);
                        if (status != null) {
                            // If a fault injection override is active, ignore the real reading and use the injected value (clearing the injection restores the real value)
                            status.batteryVoltage = (status.injectedBatteryVoltage != null)
                                    ? status.injectedBatteryVoltage : batteryVoltage;
                            status.rssi = (status.injectedRssi != null)
                                    ? status.injectedRssi : rssi;
                        }
                    }
                } catch (Exception e) {
                    System.err.println("  ⚠ Error parsing " + droneId + " status message: " + e.getMessage());
                }
            });
        }
        
        System.out.println("========================================\n");
    }
    
    /**
     * Converts world coordinates to a 3D grid index
     * 
     * Grid layout (3D):
     * - cell centre: (gridOriginX + xIndex * gridStepX, gridOriginY + yIndex * gridStepY, gridOriginZ + layerIndex * layerHeight)
     * - cell bounds: centre +/- gridStep/2 in the XY plane, +/- layerHeight/2 along Z
     * - layer selection: 0 <= z < 0.3 is layer 1, 0.3 <= z < 0.6 is layer 2, and so on
     * - formula: index = layerIndex * (bigridCols * bigridRows) + xIndex * bigridRows + yIndex
     * 
     * Example (5x5x3):
     * - layer 0: Grid[0-24], from Grid[0] at the bottom right to Grid[24] at the top left
     * - layer 1: Grid[25-49], from Grid[25] at the bottom right to Grid[49] at the top left
     * - layer 2: Grid[50-74], from Grid[50] at the bottom right to Grid[74] at the top left
     * 
     * @param x world coordinate X
     * @param y world coordinate Y
     * @param z world coordinate Z
     * @return the 0-based grid index, or -1 if out of range
     */
    private int coordinateToGridIndex(double x, double y, double z) {
        // Tolerance used to absorb floating-point precision issues
        final double EPSILON = 1e-6;
        
        // Treat -0.0 as 0.0
        if (Math.abs(x) < EPSILON) x = 0.0;
        if (Math.abs(y) < EPSILON) y = 0.0;
        if (Math.abs(z) < EPSILON) z = 0.0;
        
        // Compute the offset relative to the grid origin
        double relX = x - gridOriginX;
        double relY = y - gridOriginY;
        double relZ = z - gridOriginZ;
        
        // Compute the XY indices by rounding
        int xIndex = (int) Math.round(relX / gridStepX);
        int yIndex = (int) Math.round(relY / gridStepY);
        
        // Compute the layer index along Z, using floor to decide which layer it falls in
        // 0 <= z < 0.3 -> layer 0
        // 0.3 <= z < 0.6 -> layer 1
        // 0.6 <= z < 0.9 -> layer 2
        int layerIndex = (int) Math.floor(relZ / layerHeight);

        // Check that the indices are in range
        if (xIndex < 0 || xIndex >= bigridCols || 
            yIndex < 0 || yIndex >= bigridRows || 
            layerIndex < 0 || layerIndex >= bigridLayers) {
            return -1; // outside the grid
        }
        
        // Compute the linear 3D index
        // each layer has bigridCols * bigridRows cells
        // within a layer, X-major: cells of the same X column are listed by increasing Y
        int index = layerIndex * (bigridCols * bigridRows) + xIndex * bigridRows + yIndex;
        
        return index;
    }
    
    // /**
    //  * Converts world coordinates to a grid index (2D version, kept for compatibility)
    //  * @deprecated use the 3D version coordinateToGridIndex(x, y, z)
    //  */
    // @Deprecated
    // private int coordinateToGridIndex(double x, double y) {
    //     return coordinateToGridIndex(x, y, 0.0);
    // }
    
    // ========================================
    // Path planning and navigation
    // ========================================
    
    /**
     * Parses the target configuration and initialises the charging stations, obstacles and drone start positions
     */
    private void parseDroneTargets() {
        System.out.println("\n========================================");
        System.out.println("Parsing drone target points, charging station, and obstacles");
        System.out.println("========================================");
        
        // Initialise the obstacle grids (multiple obstacles, comma separated)
        obstacleGrids.clear();
        if (obstacleGridsConfig != null && !obstacleGridsConfig.trim().isEmpty()) {
            String[] grids = obstacleGridsConfig.split(",");
            System.out.println("  Obstacles:");
            for (String gridStr : grids) {
                try {
                    int gridIndex = Integer.parseInt(gridStr.trim());
                    GridPoint obstacle = gridIndexToPoint(gridIndex);
                    if (obstacle != null) {
                        obstacleGrids.add(gridIndex);
                        System.out.println("    - Grid[" + gridIndex + "] (" + 
                                String.format("%.1f, %.1f, %.1f", obstacle.x, obstacle.y, obstacle.z) + ")");
                    } else {
                        System.err.println("    !! Invalid obstacle grid: " + gridIndex);
                    }
                } catch (NumberFormatException e) {
                    System.err.println("    !! Invalid obstacle grid format: " + gridStr);
                }
            }
            if (obstacleGrids.isEmpty()) {
                System.out.println("    (No obstacles configured)");
            }
        } else {
            System.out.println("  (No obstacles configured)");
        }
        
        // Initialise the charging-station positions (multiple stations, comma separated)
        chargingStations.clear();
        if (chargingStationGrids != null && !chargingStationGrids.trim().isEmpty()) {
            String[] grids = chargingStationGrids.split(",");
            System.out.println("  Charging Stations:");
            for (String gridStr : grids) {
                try {
                    int gridIndex = Integer.parseInt(gridStr.trim());
                    GridPoint station = gridIndexToPoint(gridIndex);
                    if (station != null) {
                        chargingStations.add(station);
                        System.out.println("    - Grid[" + gridIndex + "] (" + 
                                String.format("%.1f, %.1f, %.1f", station.x, station.y, station.z) + ")");
                    } else {
                        System.err.println("    !! Invalid charging station grid: " + gridIndex);
                    }
                } catch (NumberFormatException e) {
                    System.err.println("    !! Invalid charging station grid format: " + gridStr);
                }
            }
            if (chargingStations.isEmpty()) {
                System.err.println("  !! No valid charging stations configured");
            }
        } else {
            System.err.println("  !! No charging stations configured");
        }
        
        String[] targets = droneTargetsConfig.split(";");
        for (int i = 0; i < targets.length && i < configuredDroneCount; i++) {
            String[] coords = targets[i].trim().split(",");
            if (coords.length == 2) {
                try {
                    double x = Double.parseDouble(coords[0].trim());
                    double y = Double.parseDouble(coords[1].trim());
                    
                    // The target uses the layer-0 Z coordinate for the grid-index calculation (targets are assumed to be on layer 0)
                    // but the actual flight altitude follows the drone's current altitude and is not changed
                    double targetZ = gridOriginZ;  // layer 0
                    int gridIndex = coordinateToGridIndex(x, y, targetZ);
                    
                    String droneId = "D" + i;
                    // The target Z is set to the layer-0 altitude (used only for the grid index, it does not affect the flight altitude)
                    droneTargets.put(droneId, new GridPoint(x, y, targetZ, gridIndex));
                    System.out.println("  " + droneId + " target: (" + x + ", " + y + ") -> Grid[" + gridIndex + "] (Layer 1)");
                    
                    // Record the drone start position from the real ROS2 position rather than from the loop index
                    DroneStatus status = droneStatuses.get(droneId);
                    DronePosition currentPos = dronePositions.get(droneId);
                    if (status != null) {
                        // Use the position reported by ROS2 as the start position
                        if (currentPos != null && currentPos.gridIndex >= 0) {
                            status.startGridIndex = currentPos.gridIndex;
                            System.out.println("  " + droneId + " start position: Grid[" + currentPos.gridIndex + "] (from ROS2)");
                        } else {
                            // Fall back to the default position if no ROS2 data has arrived yet
                            status.startGridIndex = i;
                            System.out.println("  " + droneId + " start position: Grid[" + i + "] (default)");
                        }
                        status.originalTarget = droneTargets.get(droneId);
                    }
                } catch (NumberFormatException e) {
                    System.err.println("  !! Failed to parse target point: " + targets[i]);
                }
            }
        }
        System.out.println("========================================\n");
    }
    
    /**
     * Computes the centre coordinates of a 3D grid index
     */
    private GridPoint gridIndexToPoint(int gridIndex) {
        int totalGridsPerLayer = bigridCols * bigridRows;
        if (gridIndex < 0 || gridIndex >= totalGridsPerLayer * bigridLayers) {
            return null;
        }
        
        // Compute the layer index
        int layerIndex = gridIndex / totalGridsPerLayer;
        int indexInLayer = gridIndex % totalGridsPerLayer;
        
        // Compute the XY indices
        int xIndex = indexInLayer / bigridRows;
        int yIndex = indexInLayer % bigridRows;
        
        // Compute the world coordinates
        double x = gridOriginX + xIndex * gridStepX;
        double y = gridOriginY + yIndex * gridStepY;
        double z = gridOriginZ + layerIndex * layerHeight;
        
        return new GridPoint(x, y, z, gridIndex);
    }
    
    /**
     * Returns the neighbours of a 3D grid cell (10 directions: 8 horizontal, 2 vertical)
     */
    private List<GridPoint> getNeighbors(int gridIndex) {
        List<GridPoint> neighbors = new ArrayList<>();
        
        int totalGridsPerLayer = bigridCols * bigridRows;
        int layerIndex = gridIndex / totalGridsPerLayer;
        int indexInLayer = gridIndex % totalGridsPerLayer;
        
        int xIndex = indexInLayer / bigridRows;
        int yIndex = indexInLayer % bigridRows;
        
        // 10 directions: 8 horizontal plus 2 vertical
        // horizontal directions within a layer: left, right, down, up, down-left, up-left, down-right, up-right
        int[][] horizontalDirections = {
            {-1, 0, 0},  // left
            {1, 0, 0},   // right
            {0, -1, 0},  // down
            {0, 1, 0},   // up
            {-1, -1, 0}, // down-left
            {-1, 1, 0},  // up-left
            {1, -1, 0},  // down-right
            {1, 1, 0}    // up-right
        };
        
        // Add the horizontal neighbours
        for (int[] dir : horizontalDirections) {
            int newX = xIndex + dir[0];
            int newY = yIndex + dir[1];
            int newLayer = layerIndex + dir[2];
            
            if (newX >= 0 && newX < bigridCols && 
                newY >= 0 && newY < bigridRows && 
                newLayer >= 0 && newLayer < bigridLayers) {
                int newIndex = newLayer * totalGridsPerLayer + newX * bigridRows + newY;
                GridPoint point = gridIndexToPoint(newIndex);
                if (point != null) {
                    neighbors.add(point);
                }
            }
        }
        
        // Add the vertical neighbours (the layer above and below)
        // upwards (layer + 1)
        if (layerIndex + 1 < bigridLayers) {
            int upIndex = (layerIndex + 1) * totalGridsPerLayer + xIndex * bigridRows + yIndex;
            GridPoint upPoint = gridIndexToPoint(upIndex);
            if (upPoint != null) {
                neighbors.add(upPoint);
            }
        }
        
        // downwards (layer - 1)
        if (layerIndex - 1 >= 0) {
            int downIndex = (layerIndex - 1) * totalGridsPerLayer + xIndex * bigridRows + yIndex;
            GridPoint downPoint = gridIndexToPoint(downIndex);
            if (downPoint != null) {
                neighbors.add(downPoint);
            }
        }
        
        return neighbors;
    }
    
    /**
     * Computes the 3D Euclidean distance between two points (the heuristic)
     */
    private double heuristic(GridPoint a, GridPoint b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }
    
    /**
     * Dynamic A* planning: replans the next step of the shortest path before every move
     * @param droneId drone ID
     * @param start start point
     * @param goal goal point
     * @return the next grid to move into, or null if no path can be planned
     */
    private GridPoint planNextStep(String droneId, GridPoint start, GridPoint goal) {
        if (start.gridIndex == goal.gridIndex) {
            return null;  // already at the target
        }
        
        // Collect the current obstacles (the positions of the other drones)
        Set<Integer> occupiedGrids = getOccupiedGrids(droneId);
        
        // Add the temporary avoid-set (cells recorded as blocked while detouring, expiring after 30 seconds)
        DroneStatus planStatus = droneStatuses.get(droneId);
        if (planStatus != null && !planStatus.avoidGrids.isEmpty()) {
            if (System.currentTimeMillis() - planStatus.avoidGridsSetTime < 30000) {
                occupiedGrids.addAll(planStatus.avoidGrids);
            } else {
                planStatus.avoidGrids.clear();
            }
        }
        
        // Plan the full path with A*
        List<GridPoint> fullPath = planPath(start, goal, occupiedGrids);

        // Log the full path, if one was found
        if (!fullPath.isEmpty()) {
            StringBuilder sb = new StringBuilder();
            sb.append("  [Full Path] ").append(droneId).append(": ");
            for (int i = 0; i < fullPath.size(); i++) {
                GridPoint p = fullPath.get(i);
                sb.append("Grid[").append(p.gridIndex).append("]")
                  .append("(")
                  .append(String.format(Locale.ROOT, "%.1f,%.1f,%.1f", p.x, p.y, p.z))
                  .append(")");
                if (i != fullPath.size() - 1) {
                    sb.append(" -> ");
                }
            }
            System.out.println(sb.toString());
        }
        
        if (fullPath.isEmpty() || fullPath.size() < 2) {
            return null;  // no path could be planned
        }
        
        // Return the next step of the path (index 1; index 0 is the start)
        return fullPath.get(1);
    }
    
    /**
     * A* path-planning algorithm
     * @param start start point
     * @param goal goal point
     * @param occupiedGrids the occupied grids, which must be avoided
     * @return the list of path points, from start to goal
     */
    private List<GridPoint> planPath(GridPoint start, GridPoint goal, Set<Integer> occupiedGrids) {
        PriorityQueue<AStarNode> openSet = new PriorityQueue<>();
        Set<Integer> closedSet = new HashSet<>();
        Map<Integer, Double> gScores = new HashMap<>();
        
        openSet.add(new AStarNode(start, null, 0, heuristic(start, goal)));
        gScores.put(start.gridIndex, 0.0);
        
        while (!openSet.isEmpty()) {
            AStarNode current = openSet.poll();
            
            // Goal reached
            if (current.point.gridIndex == goal.gridIndex) {
                return reconstructPath(current);
            }
            
            closedSet.add(current.point.gridIndex);
            
            // Explore the neighbours
            for (GridPoint neighbor : getNeighbors(current.point.gridIndex)) {
                // Skip nodes that have already been visited
                if (closedSet.contains(neighbor.gridIndex)) {
                    continue;
                }
                
                // Skip occupied grids, except the goal itself
                if (occupiedGrids.contains(neighbor.gridIndex) && neighbor.gridIndex != goal.gridIndex) {
                    continue;
                }
                
                // Compute the cost (in 3D, diagonal and vertical moves cost more)
                boolean isDiagonalXY = Math.abs(neighbor.x - current.point.x) > 0.5 && 
                                      Math.abs(neighbor.y - current.point.y) > 0.5;
                boolean isVertical = Math.abs(neighbor.z - current.point.z) > 0.01;
                
                double moveCost;
                if (isDiagonalXY && !isVertical) {
                    moveCost = Math.sqrt(2);  // horizontal diagonal move
                } else if (isVertical && !isDiagonalXY) {
                    moveCost = 1.5;  // vertical move (slightly more expensive, so horizontal moves are preferred)
                } else {
                    moveCost = 1.0;  // straight move
                }
                
                double tentativeGScore = current.gCost + moveCost;
                
                if (!gScores.containsKey(neighbor.gridIndex) || tentativeGScore < gScores.get(neighbor.gridIndex)) {
                    gScores.put(neighbor.gridIndex, tentativeGScore);
                    double hScore = heuristic(neighbor, goal);
                    openSet.add(new AStarNode(neighbor, current, tentativeGScore, hScore));
                }
            }
        }
        
        // No path found
        return new ArrayList<>();
    }
    
    /**
     * Reconstructs the path
     */
    private List<GridPoint> reconstructPath(AStarNode node) {
        List<GridPoint> path = new ArrayList<>();
        AStarNode current = node;
        while (current != null) {
            path.add(0, current.point);  // insert at the front
            current = current.parent;
        }
        return path;
    }
    
    /**
     * Returns every grid currently occupied by another drone, plus obstacle grids and grids reserved by other drones
     */
    private Set<Integer> getOccupiedGrids(String excludeDroneId) {
        Set<Integer> occupied = new HashSet<>();
        
        // Add the grids occupied by the other drones (their real positions)
        for (Map.Entry<String, DronePosition> entry : dronePositions.entrySet()) {
            if (!entry.getKey().equals(excludeDroneId)) {
                DronePosition pos = entry.getValue();
                if (pos.gridIndex >= 0) {
                    occupied.add(pos.gridIndex);
                }
            }
        }
        
        // Add the obstacle grids (permanently occupied, no drone may enter)
        occupied.addAll(obstacleGrids);
        
        // Add grids reserved by other drones (treated as temporary obstacles that A* routes around)
        for (Map.Entry<Integer, GridReservation> entry : gridReservations.entrySet()) {
            GridReservation reservation = entry.getValue();
            if (!reservation.droneId.equals(excludeDroneId)) {
                occupied.add(entry.getKey());
            }
        }
        
        return occupied;
    }
    
    // ========================================
    // Grid reservation system (lock based)
    // ========================================
    
    /**
     * Tries to reserve a grid
     * @param droneId drone ID
     * @param gridIndex index of the grid to reserve
     * @return true if the reservation succeeded, false if another drone already holds it
     */
    private boolean tryReserveGrid(String droneId, int gridIndex) {
        GridReservation existingReservation = gridReservations.get(gridIndex);
        
        // Already reserved
        if (existingReservation != null) {
            // Reserved by this drone - report success
            if (existingReservation.droneId.equals(droneId)) {
                return true;
            }
            // Reserved by another drone - report failure
            return false;
        }
        
        // Try to reserve it, using the atomic ConcurrentHashMap operation
        GridReservation newReservation = new GridReservation(droneId, System.currentTimeMillis());
        GridReservation previous = gridReservations.putIfAbsent(gridIndex, newReservation);
        
        // A null previous value means the reservation succeeded
        if (previous == null) {
            return true;
        }
        
        // Otherwise check whether this drone already held the reservation
        return previous.droneId.equals(droneId);
    }
    
    /**
     * Releases a grid reservation
     * @param droneId drone ID
     * @param gridIndex index of the grid to release
     */
    private void releaseGrid(String droneId, int gridIndex) {
        GridReservation reservation = gridReservations.get(gridIndex);
        if (reservation != null && reservation.droneId.equals(droneId)) {
            gridReservations.remove(gridIndex);
        }
    }
    
    /**
     * Releases every grid reservation held by a drone
     * @param droneId drone ID
     */
    private void releaseAllGrids(String droneId) {
        gridReservations.entrySet().removeIf(entry -> entry.getValue().droneId.equals(droneId));
    }
    
    /**
     * Checks whether a grid is reserved
     * @param gridIndex grid index
     * @return the ID of the drone holding the reservation, or null if the grid is free
     */
    private String getGridReservation(int gridIndex) {
        GridReservation reservation = gridReservations.get(gridIndex);
        return (reservation != null) ? reservation.droneId : null;
    }
    
    // ========================================
    // Collision detection
    // ========================================
    
    /**
     * Detects a collision risk: two or more drones in the same grid
     */
    private void checkCollisionRisk() {
        Map<Integer, List<String>> gridOccupancy = new HashMap<>();
        
        for (DronePosition pos : dronePositions.values()) {
            if (pos.gridIndex >= 0) {
                gridOccupancy.computeIfAbsent(pos.gridIndex, k -> new ArrayList<>())
                        .add(pos.droneId);
            }
        }
        
        Set<Integer> currentViolatingGrids = new HashSet<>();
        boolean riskDetected = false;
        Map<Integer, List<String>> violatingOccupants = new HashMap<>();
        for (Map.Entry<Integer, List<String>> entry : gridOccupancy.entrySet()) {
            if (entry.getValue().size() > 1) {
                riskDetected = true;
                currentViolatingGrids.add(entry.getKey());
                violatingOccupants.put(entry.getKey(), entry.getValue());
                if (!collisionRiskDetected.get()) {
                    System.err.println("\n⚠⚠⚠ Collision Risk Warning (potential, pre-debounce) ⚠⚠⚠");
                    System.err.println("Multiple drones in same grid:");
                }
                System.err.println("  Grid[" + entry.getKey() + "]: " +
                        String.join(", ", entry.getValue()));
            }
        }

        // Debounce plus flight-phase filtering: a violation is counted only when a cell is occupied by >=2 airborne drones for violationDebounceFrames consecutive frames,
        // which rules out two kinds of false overlap: (a) single-frame flicker from boundary jitter or asynchronous ghosting;
        // (b) overlaps on the ground before take-off or during simulation start-up (exclusion is a property of the flight phase, ground spawn overlaps do not count).
        for (Integer gridIndex : currentViolatingGrids) {
            long airborneOccupants = violatingOccupants.get(gridIndex).stream()
                    .filter(id -> {
                        DroneStatus s = droneStatuses.get(id);
                        return s != null && s.hasTakenOff;
                    })
                    .count();
            if (airborneOccupants < 2) {
                // Fewer than two airborne drones overlap in this cell (on the ground or before take-off): no violation, reset its consecutive-frame counter.
                // Note: the riskDetected flag and the merge pause above still use the raw detection, staying conservative.
                coOccupancyStreak.remove(gridIndex);
                continue;
            }
            int streak = coOccupancyStreak.merge(gridIndex, 1, Integer::sum);
            if (streak >= violationDebounceFrames && !countedViolationGrids.contains(gridIndex)) {
                long total = coOccupancyViolationCount.incrementAndGet();
                countedViolationGrids.add(gridIndex);
                System.err.println("  ✗ CONFIRMED co-occupancy violation at Grid[" + gridIndex + "]: "
                        + String.join(", ", violatingOccupants.get(gridIndex))
                        + " (sustained " + streak + " frames ≥ " + violationDebounceFrames
                        + "; total confirmed = " + total + ")");
            }
        }
        // Cells whose conflict has ended: reset the consecutive-frame counter and clear the counted flag, so the next sustained conflict is counted again.
        coOccupancyStreak.keySet().retainAll(currentViolatingGrids);
        countedViolationGrids.retainAll(currentViolatingGrids);

        if (riskDetected && !collisionRiskDetected.get()) {
            System.err.println("Temporarily not merging models to avoid conflict");
            System.err.println("⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠\n");
        }

        collisionRiskDetected.set(riskDetected);
    }
    
    private void recordRosPoseSample() {
        long now = System.currentTimeMillis();
        if (firstRosPoseTimeMs == null) {
            firstRosPoseTimeMs = now;
        }
        lastRosPoseTimeMs = now;
    }
    
    /** Gate: only calls writeMissionReport("complete") once every configured drone has landed and stayed landed for LANDING_DELAY_MS. */
    private void maybePrintMissionReport() {
        if (missionReportPrinted.get()) {
            return;
        }

        for (int i = 0; i < configuredDroneCount; i++) {
            DroneStatus status = droneStatuses.get("D" + i);
            if (status == null || !status.hasLanded) {
                allDronesLandedAtMs = null;
                return;
            }
        }

        long now = System.currentTimeMillis();
        if (allDronesLandedAtMs == null) {
            allDronesLandedAtMs = now;
            return;
        }
        if (now - allDronesLandedAtMs < LANDING_DELAY_MS) {
            return;
        }

        writeMissionReport("complete");
    }

    /**
     * Builds and writes the mission report (completion time, violations, per-decision latencies)
     * from whatever metrics have been collected so far. Guarded by missionReportPrinted so it runs
     * at most once, whether triggered by normal completion or by the JVM shutdown hook (Ctrl+C).
     * @param runStatus "complete" (all drones landed) or "interrupted" (JVM exiting early)
     */
    private void writeMissionReport(String runStatus) {
        if (!missionReportPrinted.compareAndSet(false, true)) {
            return;
        }

        double completionTimeSec = 0.0;
        if (firstRosPoseTimeMs != null && lastRosPoseTimeMs != null) {
            completionTimeSec = (lastRosPoseTimeMs - firstRosPoseTimeMs) / 1000.0;
        }

        String modeLabel = baselineMode ? "BASELINE" : "BIGRAPH";

        // Each stats block has the shape {"overall": {...pooled...}, "by_drone": {"D0": {...}, "D1": {...}, ...}}
        // so per-drone samples (e.g. cf231/D0 vs cf232/D1) are never pooled together at the source.
        Map<String, Object> planningStats = summarizeByDrone(planningLatenciesMsByDrone);
        Map<String, Object> gateStats = summarizeByDrone(gateMatchLatenciesMsByDrone);
        Map<String, Object> restStats = summarizeByDrone(restLatenciesMsByDrone);

        System.out.println("\n========== MISSION REPORT (" + modeLabel + ", " + runStatus + ") ==========");
        System.out.println("Co-occupancy violations: " + coOccupancyViolationCount.get());
        System.out.println("Completion time: " + String.format(Locale.ROOT, "%.2f", completionTimeSec) + " s");
        if (firstRosPoseTimeMs != null && lastRosPoseTimeMs != null) {
            System.out.println("  (ROS pose span: t0=" + firstRosPoseTimeMs + " t1=" + lastRosPoseTimeMs + ")");
        }
        System.out.println("Per-decision latency (ms), pooled across all drones:");
        System.out.println("  Planning : " + formatLatencySummary(overallOf(planningStats)));
        System.out.println("  Gate     : " + formatLatencySummary(overallOf(gateStats)));
        System.out.println("  REST     : " + formatLatencySummary(overallOf(restStats)));
        System.out.println("Per-drone gate (Match) latency (ms):");
        for (Map.Entry<String, Object> e : byDroneOf(gateStats).entrySet()) {
            @SuppressWarnings("unchecked")
            Map<String, Object> droneStats = (Map<String, Object>) e.getValue();
            System.out.println("  " + e.getKey() + " : " + formatLatencySummary(droneStats));
        }
        System.out.println("====================================================\n");

        String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
        String reportFileName = "mission-report-" + modeLabel.toLowerCase(Locale.ROOT) + "-" + runStatus + "-" + timestamp + ".json";
        try {
            Map<String, Object> report = new LinkedHashMap<>();
            report.put("run_status", runStatus);
            report.put("generated_at", timestamp);
            report.put("baseline_mode", baselineMode);
            report.put("co_occupancy_violations", coOccupancyViolationCount.get());
            report.put("completion_time_sec", completionTimeSec);
            report.put("first_ros_pose_ms", firstRosPoseTimeMs);
            report.put("last_ros_pose_ms", lastRosPoseTimeMs);
            report.put("planning_latency_ms", planningStats);
            report.put("gate_match_latency_ms", gateStats);
            report.put("rest_latency_ms", restStats);
            objectMapper.writerWithDefaultPrettyPrinter()
                    .writeValue(new java.io.File(reportFileName), report);
            System.out.println("✓ Mission report written to " + reportFileName);
        } catch (Exception e) {
            System.err.println("!! Failed to write " + reportFileName + ": " + e.getMessage());
        }
    }

    /** Computes count/mean/p50/p95/max (ms) from a collection of per-decision latency samples. */
    private static Map<String, Object> summarizeLatenciesMs(Collection<Double> samples) {
        Map<String, Object> stats = new LinkedHashMap<>();
        if (samples.isEmpty()) {
            stats.put("count", 0);
            stats.put("mean_ms", null);
            stats.put("p50_ms", null);
            stats.put("p95_ms", null);
            stats.put("max_ms", null);
            return stats;
        }
        List<Double> sorted = new ArrayList<>(samples);
        Collections.sort(sorted);
        double mean = sorted.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        stats.put("count", sorted.size());
        stats.put("mean_ms", mean);
        stats.put("p50_ms", percentile(sorted, 0.50));
        stats.put("p95_ms", percentile(sorted, 0.95));
        stats.put("max_ms", sorted.get(sorted.size() - 1));
        return stats;
    }

    /** Nearest-rank percentile over an ascending-sorted list. */
    private static double percentile(List<Double> sortedAscending, double p) {
        int idx = (int) Math.ceil(p * sortedAscending.size()) - 1;
        idx = Math.max(0, Math.min(sortedAscending.size() - 1, idx));
        return sortedAscending.get(idx);
    }

    private static String formatLatencySummary(Map<String, Object> stats) {
        int count = (int) stats.get("count");
        if (count == 0) {
            return "n=0 (no samples)";
        }
        return String.format(Locale.ROOT, "n=%d  mean=%.3f  p50=%.3f  p95=%.3f  max=%.3f",
                count, (double) stats.get("mean_ms"), (double) stats.get("p50_ms"),
                (double) stats.get("p95_ms"), (double) stats.get("max_ms"));
    }

    /**
     * Builds {"overall": pooled-summary, "by_drone": {droneId: summary, ...}} from a per-drone
     * sample store, so each drone's latency distribution stays separately inspectable (never
     * silently merged with another drone's), while still exposing a pooled figure for convenience.
     */
    private static Map<String, Object> summarizeByDrone(Map<String, Queue<Double>> byDrone) {
        List<Double> pooled = new ArrayList<>();
        for (Queue<Double> samples : byDrone.values()) {
            pooled.addAll(samples);
        }

        List<String> droneIds = new ArrayList<>(byDrone.keySet());
        droneIds.sort(Comparator.comparingInt(Application3D::droneIndexOf));

        Map<String, Object> perDrone = new LinkedHashMap<>();
        for (String droneId : droneIds) {
            perDrone.put(droneId, summarizeLatenciesMs(byDrone.get(droneId)));
        }

        Map<String, Object> result = new LinkedHashMap<>();
        result.put("overall", summarizeLatenciesMs(pooled));
        result.put("by_drone", perDrone);
        return result;
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> overallOf(Map<String, Object> byDroneStats) {
        return (Map<String, Object>) byDroneStats.get("overall");
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> byDroneOf(Map<String, Object> byDroneStats) {
        return (Map<String, Object>) byDroneStats.get("by_drone");
    }

    /** Extracts the numeric index from a droneId like "D3" -> 3, for stable console/JSON ordering. */
    private static int droneIndexOf(String droneId) {
        try {
            return Integer.parseInt(droneId.substring(1));
        } catch (NumberFormatException | IndexOutOfBoundsException e) {
            return Integer.MAX_VALUE;
        }
    }
    
    /**
     * Updates the drone model from the live ROS2 positions
     * @param siteCount number of grid sites
     * @return the updated drone model
     */
    private PureBigraph droneModelFromRosPositions(int siteCount) throws InvalidConnectionException, TypeNotExistsException, IncompatibleSignatureException, IncompatibleInterfaceException {
        if (!rosUpdateEnabled || dronePositions.isEmpty()) {
            // Fall back to the default placement if ROS2 is disabled or no position data is available
            return droneModel(siteCount);
        }
        
        // Check for a collision risk
        if (collisionRiskDetected.get()) {
            System.err.println("⚠ Collision risk detected, using previous drone model");
            return dronePart; // return the current model without updating it
        }
        
        // Build the list of empty sites, including the obstacles
        List<Bigraph<DynamicSignature>> placements = new ArrayList<>();
        for (int i = 0; i < siteCount; i++) {
            // Check whether this is an obstacle grid
            if (obstacleGrids.contains(i)) {
                placements.add(buildObstacleCell());
            } else {
                placements.add(emptyOccupiedCell());
            }
        }
        
        // Place the drones according to their ROS2 positions, using their real status
        // Iterate over every value in the dronePositions map; pos holds droneId, x, y and gridIndex
        Set<Integer> placedCells = new HashSet<>();
        for (DronePosition pos : dronePositions.values()) {
            int placeIndex = pos.gridIndex;

            // Positional drift is tolerated only before take-off (while hasTakenOff is false): on grid=-1 fall back to the start cell,
            // which keeps the drone in the model and lets it take off normally (controlled by takeoff.allow-out-of-grid, and can be disabled).
            // After take-off there is no fallback: a persistent grid=-1 in flight keeps the model strictly consistent with the physical state and surfaces the real problem.
            if ((placeIndex < 0 || placeIndex >= siteCount) && allowOutOfGridPlacement) {
                DroneStatus st = droneStatuses.get(pos.droneId);
                boolean notTakenOff = (st == null) || !st.hasTakenOff;
                if (notTakenOff) {
                    int fallback = fallbackGridIndex(pos.droneId, siteCount);
                    if (fallback >= 0 && !placedCells.contains(fallback) && !obstacleGrids.contains(fallback)) {
                        System.out.println("  ⚠ " + pos.droneId + " out-of-grid (grid=" + pos.gridIndex
                                + ", sensor drift, pre-takeoff), placing at fallback Grid[" + fallback + "] to allow takeoff");
                        placeIndex = fallback;
                    }
                }
            }

            if (placeIndex >= 0 && placeIndex < siteCount) {
                // Skip obstacle grids; do not place a drone on an obstacle
                if (obstacleGrids.contains(placeIndex)) {
                    continue;
                }

                // Get the actual drone status
                String droneStatus = "Landed";  // Default status
                DroneStatus status = droneStatuses.get(pos.droneId);
                if (status != null) {
                    if (status.landingRuleApplied) {
                        droneStatus = "Landed";  // the landing rule has been applied
                    } else if (status.takeoffRuleApplied) {
                        droneStatus = "flying";  // the take-off rule has been applied
                    }
                }
                // Place the drone at the given position
                placements.set(placeIndex,
                        buildDrone(pos.droneId, droneStatus, "OccupiedBy"));
                placedCells.add(placeIndex);
            }
        }
        
        Bigraph<DynamicSignature> result = placements.stream()
                .reduce(pureLinkings(sig()).identity_e(), accumulator::apply);
        return (PureBigraph) result;
    }

    /**
     * Fallback cell used when the position falls outside the grid (grid=-1): the drone's start cell first, otherwise its number (D{idx} -> cell idx).
     * Returns -1 when there is no valid fallback cell.
     */
    private int fallbackGridIndex(String droneId, int siteCount) {
        DroneStatus status = droneStatuses.get(droneId);
        if (status != null && status.startGridIndex != null
                && status.startGridIndex >= 0 && status.startGridIndex < siteCount) {
            return status.startGridIndex;
        }
        try {
            int idx = Integer.parseInt(droneId.substring(1));
            if (idx >= 0 && idx < siteCount) {
                return idx;
            }
        } catch (NumberFormatException ignored) {
            // droneId is not in the "D<number>" form, so there is no numeric fallback
        }
        return -1;
    }

    /**
     * Drone position information (3D)
     */
    private static class DronePosition {
        final String droneId;
        final double x;
        final double y;
        final double z;
        final int gridIndex;
        
        DronePosition(String droneId, double x, double y, double z, int gridIndex) {
            this.droneId = droneId;
            this.x = x;
            this.y = y;
            this.z = z;
            this.gridIndex = gridIndex;
        }
    }
    
    private static class DroneStatus {
        final String droneId;
        // Flight state
        volatile double z;
        volatile String status = "Landed";
        volatile boolean hasTakenOff, takeoffRuleApplied, takeoffCommandSent;
        volatile long takeoffTime;
        volatile boolean landingCommandSent, hasLanded, landingRuleApplied;
        volatile long landingCommandTime;
        // Navigation state
        volatile boolean reachedDestination, isWaiting, isMoving;
        volatile long lastMoveTime, waitingStartTime;
        volatile Integer waitingForGrid, reservedGrid, movingToGrid;
        // Status monitoring
        volatile double batteryVoltage = 4.2;
        volatile int rssi;
        volatile String batteryLevel = "Normal", communicationStatus = "Normal";
        // Fault-injection override (null = use the real ROS reading). When non-null, the value reported by ROS is ignored in favour of the injected one.
        volatile Double injectedBatteryVoltage = null;
        volatile Integer injectedRssi = null;
        volatile boolean batteryRuleApplied, communicationRuleApplied, bigraphBatteryNeedsSync;
        // Emergency
        volatile boolean isInEmergency, waitingForRecharge, emergencyLanded;
        volatile String emergencyType;
        volatile GridPoint originalTarget, emergencyTarget;
        volatile Integer startGridIndex;
        // Deadlock avoidance: grids to temporarily avoid during path planning
        final Set<Integer> avoidGrids = ConcurrentHashMap.newKeySet();
        volatile long avoidGridsSetTime;
        volatile int consecutiveWaitCycles;
        
        DroneStatus(String droneId) { this.droneId = droneId; }
    }
    
    // 3D grid point
    private static class GridPoint {
        final double x;
        final double y;
        final double z;
        final int gridIndex;
        
        GridPoint(double x, double y, double z, int gridIndex) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.gridIndex = gridIndex;
        }
        
        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            GridPoint that = (GridPoint) o;
            return gridIndex == that.gridIndex;
        }
        
        @Override
        public int hashCode() {
            return Integer.hashCode(gridIndex);
        }
        
        @Override
        public String toString() {
            return String.format("Grid[%d](%.1f,%.1f,%.1f)", gridIndex, x, y, z);
        }
    }
    
    // A* search node
    private static class AStarNode implements Comparable<AStarNode> {
        final GridPoint point;
        final AStarNode parent;
        final double gCost;  // actual cost from the start to this point
        final double hCost;  // heuristic cost from this point to the goal
        final double fCost;  // gCost + hCost
        
        AStarNode(GridPoint point, AStarNode parent, double gCost, double hCost) {
            this.point = point;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
        }
        
        @Override
        public int compareTo(AStarNode other) {
            return Double.compare(this.fCost, other.fCost);
        }
    }
    
    // Grid reservation record
    private static class GridReservation {
        final String droneId;           // ID of the drone holding the reservation
        final long reservationTime;     // time the reservation was made
        
        GridReservation(String droneId, long reservationTime) {
            this.droneId = droneId;
            this.reservationTime = reservationTime;
        }
    }
}
