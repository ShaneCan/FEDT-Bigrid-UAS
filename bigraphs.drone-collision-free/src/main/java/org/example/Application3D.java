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
    /** 移动超时：若发送导航指令后超过此时间仍未到达 movingToGrid，则放弃等待并从当前位置重新规划 */
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

    /** A* 路径规划 + 斜向/碰撞安全检查（不含 Bigraph match、预订、REST、sleep） */
    private void logPlanningLatency(String droneId, String action, long startNano, long endNano) {
        double ms = elapsedMs(startNano, endNano);
        recordLatency(planningLatenciesMsByDrone, droneId, ms);
        System.out.println("  ⏱ [Planning] " + droneId + " " + action + ": "
                + String.format(Locale.ROOT, "%.3f", ms) + " ms");
    }

    /** 仅 Bigraph match（无路径规划，不含 CDO 持久化）—— 即 FEDT runtime gate 自身的判定耗时 */
    private void logMatchOnlyLatency(String droneId, String action, long startNano, long endNano) {
        double ms = elapsedMs(startNano, endNano);
        recordLatency(gateMatchLatenciesMsByDrone, droneId, ms);
        System.out.println("  ⏱ [Match] " + droneId + " " + action + ": "
                + String.format(Locale.ROOT, "%.3f", ms) + " ms");
    }

    /** REST：httpClient.send 调用前至收到响应（不含 sleep） */
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

    private static final double BATTERY_EMPTY_THRESHOLD = 2.1; //3.1！！！！！！！！！
    private static final double BATTERY_LOW_THRESHOLD = 2.2; //3.2！！！！！！！！！

    // RSSI threshold
    private static final int RSSI_BAD_THRESHOLD = 85;//65！！！！！！！！！

    // 故障注入用的合成读数（相对于上面的阈值选取，注入这些值即可触发对应的规则/应急响应）
    private static final double INJECT_BATTERY_NORMAL_V = 4.0;   // >= BATTERY_LOW_THRESHOLD
    private static final double INJECT_BATTERY_LOW_V    = 3.15;  // [EMPTY, LOW) -> Low
    private static final double INJECT_BATTERY_EMPTY_V  = 3.0;  // < EMPTY     -> Empty
    private static final int    INJECT_RSSI_BAD    = 70;         // >= RSSI_BAD_THRESHOLD -> Bad
    private static final int    INJECT_RSSI_NORMAL = 50;         // <  RSSI_BAD_THRESHOLD -> Normal
    @Autowired
    protected CdoTemplate template;

    @Value("${bigrid.service.base-url:http://127.0.0.1:7070}")
    private String bigridServiceBaseUrl;

    @Value("${bigrid.service.rows:7}")
    private int bigridRows;

    @Value("${bigrid.service.cols:7}")
    private int bigridCols;
    
    @Value("${bigrid.service.layers:3}")
    private int bigridLayers;

    @Value("${bigrid.service.format:xml}")
    private String bigridFormat;

    @Value("${bigrid.service.origin.x:-1.5}")
    private double gridOriginX;

    @Value("${bigrid.service.origin.y:-1.5}")
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

    // 是否启用仿真模式：位置来自 /cf_positions_poses（PoseArray），而不是每个 /cfXXX/pose
    @Value("${ros.sim.mode:true}")
    private boolean rosSimMode;

    // Drone Control Configuration
    @Value("${drone.control.enabled:true}")
    private boolean droneControlEnabled;
    @Value("${drone.control.base-url:http://127.0.0.1}")
    private String droneControlBaseUrl;
    @Value("${drone.control.start-port:5000}")
    private int droneControlStartPort;

    // 无人机目标点配置 (格式: "x1,y1;x2,y2;...")
    @Value("${drone.targets:-1,-1.5;1,-1.5;1.5,-1;1.5,1;1,1.5;-1,1.5;-1.5,1;-1.5,-1;-1,-1;1,-1;1,1;-1,1}")
    private String droneTargetsConfig;
    
    // 充电站配置（支持多个充电站，用逗号分隔，例如：3,8,13）
    @Value("${charging.station.grids:3}")
    private String chargingStationGrids;
    
    // 障碍物栅格配置（支持多个障碍物，用逗号分隔，例如：15,20,25）
    @Value("${obstacle.grids:18,67}")
    private String obstacleGridsConfig;

    @Value("${baseline.mode:false}")
    private boolean baselineMode;

    // 故障注入 HTTP 控制端口：运行时可随时通过 curl 让某架无人机 battery Low/Empty 或 comm Bad
    @Value("${fault.injection.enabled:true}")
    private boolean faultInjectionEnabled;
    @Value("${fault.injection.port:8090}")
    private int faultInjectionPort;

    // 集体升高（Collective Ascent）：起飞后把所有无人机统一抬升到一个工作高度再开始栅格导航。
    // 用途：满足垂直下洗流间隔（Preiss et al. IROS 2017，中心间距≥0.6m）。Crazyflie 起飞默认约0.6m，
    // 抬升到 0.9m 后开始运行
    @Value("${collective.ascent.enabled:true}")
    private boolean collectiveAscentEnabled;
    // 目标绝对工作高度（米）。
    @Value("${collective.ascent.altitude:0.9}")
    private double collectiveAscentAltitude;
    // 等待所有无人机完成起飞的超时（毫秒）
    @Value("${collective.ascent.takeoff-wait-timeout-ms:8000}")
    private long collectiveAscentTakeoffWaitMs;
    // 发送升高指令后的稳定等待（毫秒），让 status.z 收敛到目标高度
    @Value("${collective.ascent.settle-ms:4000}")
    private long collectiveAscentSettleMs;

    // 垂直导航绝对高度模式：目标高度 = base-altitude + 目标层*layerHeight，与当前物理高度无关，
    // 消除"相对 status.z ± layerHeight"造成的逐层漂移。推荐配合 collective.ascent 使用。
    @Value("${navigation.absolute-altitude.enabled:true}")
    private boolean absoluteAltitudeEnabled;
    // layer 0 的物理工作高度（米），仅在 absolute-altitude 模式下使用。
    // 例：base-altitude=0.3、layerHeight=0.6 → layer0=0.3m, layer1=0.9m, layer2=1.5m。
    @Value("${navigation.base-altitude:0.3}")
    private double navigationBaseAltitude;

    // 传感器漂移导致位置落在栅格外(grid=-1)时，是否回退到起始格放置无人机，
    // 使其仍留在模型中并允许正常起飞（避免定位抖动导致起飞规则不匹配）。实验用，可随时设为 false 关闭。
    @Value("${takeoff.allow-out-of-grid:true}")
    private boolean allowOutOfGridPlacement;

    // 同格共占违规去抖阈值：某格需连续这么多帧被≥2机占用，才计一次真违规。
    // 用于过滤边界抖动/异步残影造成的单帧假重叠。设为 1 则等同不去抖（旧行为）。
    @Value("${collision.violation.debounce-frames:3}")
    private int violationDebounceFrames;

    private final HttpClient httpClient = HttpClient.newBuilder()
            .connectTimeout(Duration.ofSeconds(5))
            .build();
    
    // 无人机目标点映射
    private final Map<String, GridPoint> droneTargets = new HashMap<>();
    
    // 充电站位置列表（支持多个充电站）
    private List<GridPoint> chargingStations = new ArrayList<>();
    
    // 障碍物栅格索引集合（线程安全）
    private final Set<Integer> obstacleGrids = ConcurrentHashMap.newKeySet();

    private final ObjectMapper objectMapper = new ObjectMapper();
    
    // 无人机位置信息存储（线程安全）：droneId -> {x, y, gridIndex}
    private final Map<String, DronePosition> dronePositions = new ConcurrentHashMap<>();
    
    // 无人机状态信息存储（线程安全）：droneId -> {status, z, hasTakenOff}
    private final Map<String, DroneStatus> droneStatuses = new ConcurrentHashMap<>();
    
    // 碰撞风险标记
    private final AtomicBoolean collisionRiskDetected = new AtomicBoolean(false);
    
    // Grid预订系统：防止多个无人机同时移动到同一个grid
    private final Map<Integer, GridReservation> gridReservations = new ConcurrentHashMap<>();
    
    // CDO操作同步锁：防止多个线程同时操作CDO数据库导致冲突
    private final Object cdoLock = new Object();

    private DynamicSignature combinedSignature;
    private DynamicSignature serviceWorldSignature;  // 从服务World Model提取的签名
    
    // CDO 更新策略：保存对象 ID，用于后续更新
    private CDOID cdoIdWorld;
    private CDOID cdoIdDrone;
    private CDOID cdoIdComposed;
    
    // 保存当前的 Bigraph 对象引用
    private PureBigraph worldPart;
    private PureBigraph dronePart;
    private PureBigraph composite;
    
    // 标记位：是否已完成首次模型更新
    private final AtomicBoolean firstUpdateCompleted = new AtomicBoolean(false);

    // Mission metrics (Baseline + Bigraph runs use the same ROS pose span for completion time)
    private final AtomicLong coOccupancyViolationCount = new AtomicLong(0);
    // 去抖：记录每个格子"连续被≥2机占用"的帧数；只有连续帧数达到阈值才计入一次真违规，
    // 过滤边界抖动/异步残影造成的单帧假重叠。
    private final Map<Integer, Integer> coOccupancyStreak = new ConcurrentHashMap<>();
    // 本次持续冲突段中已计过数的格子（清空后同一格再次冲突才会重新计数）。
    private final Set<Integer> countedViolationGrids = ConcurrentHashMap.newKeySet();

    // Per-decision, per-drone latency samples (ms), disjoint phases: Planning (A* + diagonal check) ->
    // Match (FEDT runtime gate admissibility check only) -> REST (dispatch to drone control API).
    // Keyed by droneId so cf231/D0's samples are never pooled with cf232/D1's, etc.
    // Populated inside logPlanningLatency / logMatchOnlyLatency / logRestLatency respectively.
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

        // 步骤1: 首先获取World签名（从本地文件，与服务兼容）
        fetchWorldSignatureFromService();
        
        // 步骤2: 构建合并签名（World + Drone）
        System.out.println("Building merged signature");
        sig();

        // 步骤3: 加载World Model（使用world签名）
        worldPart = fetchWorldModelFromService();
        
        // 步骤4: 注册元模型到CDO
        registerMetaModelToCDO();

        System.out.println("\n========================================");
        System.out.println("Creating Drone Model and Composite Model");
        System.out.println("========================================");
        
        // 步骤5: 创建Drone Model和组合模型
        int currentSiteCount = worldPart.getSites().size();
        dronePart = droneModel(currentSiteCount);
        composite = composeWorldAndDrones(worldPart, dronePart);

        // 步骤6: 插入对象并保存 CDOID，用于后续更新（保留历史版本），这一步需要元模型信息，所以必须先注册元模型（步骤4）
        EPackage MM = createOrGetBigraphMetaModel(sig());
        worldPart = BigraphUtil.toBigraph(MM, template.insert(worldPart.getInstanceModel(), "/world"), sig());
        dronePart = BigraphUtil.toBigraph(MM, template.insert(dronePart.getInstanceModel(), "/drone"), sig());
        composite = BigraphUtil.toBigraph(MM, template.insert(composite.getInstanceModel(), "/composed"), sig());

        // 步骤7: 保存 CDOID，用于后续更新
        cdoIdWorld = CDOUtil.getCDOObject(worldPart.getInstanceModel()).cdoID();
        cdoIdDrone = CDOUtil.getCDOObject(dronePart.getInstanceModel()).cdoID();
        cdoIdComposed = CDOUtil.getCDOObject(composite.getInstanceModel()).cdoID();
        
        System.out.println("✓ World Model inserted into CDO (CDOID: " + cdoIdWorld + ")");
        System.out.println("✓ Drone Model inserted into CDO (CDOID: " + cdoIdDrone + ")");
        System.out.println("✓ Composite Model inserted into CDO (CDOID: " + cdoIdComposed + ")");
        
        // 初始化 ROS2 订阅（如果启用）
        initializeRosSubscriptions();

        // 启动故障注入 HTTP 控制端点（可随时 curl 让某架无人机 battery Low/Empty 或 comm Bad）
        startFaultInjectionServer();

        if (!baselineMode) {
            // 启动后台模型更新线程（持续快速更新）
            Thread modelUpdateThread = new Thread(this::continuousModelUpdate, "ModelUpdateThread");
            modelUpdateThread.setDaemon(false);
            modelUpdateThread.start();
            
            // 等待第一次模型更新完成
            System.out.println("Waiting for first model update to complete...");
            while (!firstUpdateCompleted.get()) {
                TimeUnit.MILLISECONDS.sleep(100);
            }
            System.out.println("✓ First model update completed\n");
        } else {
            firstUpdateCompleted.set(true);
            System.out.println("✓ Baseline mode: skipping continuous Bigraph model update thread\n");
        }
        
        // 启动电池和通信状态监测线程（如果启用ROS2）
        if (rosUpdateEnabled) {
            Thread statusMonitorThread = new Thread(this::continuousStatusMonitoring, "StatusMonitorThread");
            statusMonitorThread.setDaemon(false);
            statusMonitorThread.start();
            System.out.println("✓ Battery and communication status monitoring thread started");
        }
        
        // 等待ROS2状态数据稳定（电池电压、RSSI等）
        // 这确保Pre-Takeoff Check时能获取到真实的电池状态
        if (rosUpdateEnabled) {
            System.out.println("Waiting for ROS2 status data to stabilize (3 seconds)...");
            TimeUnit.MILLISECONDS.sleep(3000);
            System.out.println("✓ ROS2 status data should be stable now\n");
        }
        
        // 解析目标点配置（在ROS2数据稳定后解析，以获取正确的起始位置）
        parseDroneTargets();
        
        // 执行起飞序列：规则匹配并发送起飞指令
        if (droneControlEnabled) {
            performTakeoffSequence();
            // 起飞后集体升高到统一工作高度（如启用），再进入导航循环
            performCollectiveAscent();
        }
        
        System.out.println("\n========================================");
        System.out.println("Entering navigation control loop...");
        System.out.println("========================================\n");
        
        // 主控制循环：处理起飞和降落规则应用
        while (true) {
            TimeUnit.MILLISECONDS.sleep(1000);  // 每秒检查一次
            
            if (!droneControlEnabled || !rosUpdateEnabled) {
                continue;
            }
            
            // 检查并应用起飞规则（当无人机实际起飞后）
            checkAndApplyTakeoffRules();
            
            // 执行导航控制（规划路径并移动）
            if (baselineMode) {
                performBaselineNavigationControl();
            } else {
                performNavigationControl();
            }
            
            // 检查并应用降落规则（到达目标后降落）
            checkAndApplyLandingRules();
            
            maybePrintMissionReport();
        }
    }
    
    /**
     * 持续快速更新模型（后台线程）
     * ContinuousModelUpdate添加同步锁，防止与StatusMonitorThread同时操作CDO导致冲突
     */
    private void continuousModelUpdate() {
        try {
            int currentSiteCount = worldPart.getSites().size();
            int updateCount = 0;
            
            while (true) {
                updateCount++;
                
                // 获取最新的 World Model（这个操作不需要锁，只是HTTP请求）
                PureBigraph latestWorld = fetchWorldModelFromService();
                int newSiteCount = latestWorld.getSites().size();
                
                // 根据 ROS2 位置或站点数量变化，重新创建 Drone Model
                boolean needUpdateDrone = (newSiteCount != currentSiteCount) || 
                                         (rosUpdateEnabled && !dronePositions.isEmpty() && !collisionRiskDetected.get());
                
                //使用同步锁包裹所有CDO操作
                synchronized (cdoLock) {
                    if (needUpdateDrone) {
                        if (newSiteCount != currentSiteCount) {
                            System.out.println("  [Update" + updateCount + "] Site count changed: " + currentSiteCount + " -> " + newSiteCount);
                            currentSiteCount = newSiteCount;
                        }
                        
                        // 根据 ROS2 位置创建 Drone Model（如果启用且无碰撞风险）
                        if (rosUpdateEnabled && !dronePositions.isEmpty() && !collisionRiskDetected.get()) {
                            dronePart = droneModelFromRosPositions(newSiteCount);
                        } else {
                            dronePart = droneModel(newSiteCount);
                        }
                        
                        // 更新 Drone Model：插入新版本（CDO审计功能会保留历史）
                        EObject insertedDrone = template.insert(dronePart.getInstanceModel(), "/drone");
                        dronePart = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedDrone, sig());
                        cdoIdDrone = CDOUtil.getCDOObject(dronePart.getInstanceModel()).cdoID();
                    }

                    // 更新 World Model：插入新版本（CDO审计功能会保留历史）
                    EObject insertedWorld = template.insert(latestWorld.getInstanceModel(), "/world");
                    worldPart = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedWorld, sig());
                    cdoIdWorld = CDOUtil.getCDOObject(worldPart.getInstanceModel()).cdoID();

                    // 更新 Composite Model：插入新版本（CDO审计功能会保留历史）
                    PureBigraph updatedComposite = composeWorldAndDrones(worldPart, dronePart);
                    EObject insertedComposite = template.insert(updatedComposite.getInstanceModel(), "/composed");
                    composite = BigraphUtil.toBigraph(createOrGetBigraphMetaModel(sig()), insertedComposite, sig());
                    cdoIdComposed = CDOUtil.getCDOObject(composite.getInstanceModel()).cdoID();
                }
                
                // 标记首次更新完成
                if (!firstUpdateCompleted.get()) {
                    firstUpdateCompleted.set(true);
                }
                
                // ##############模型更新固定延迟，毫秒。
                TimeUnit.MILLISECONDS.sleep(500);
            }
        } catch (Exception e) {
            System.err.println("!! Model update thread exception: " + e.getMessage());
            e.printStackTrace();
        }
    }
    

    private void prepareDatabase() throws Exception {
        System.out.println("Preparing CDO database...");
        
        // 注意：此时签名还未构建，只是清理数据库，元模型注册将在后面进行
        
        // 安全地删除资源：捕获所有异常以处理脏资源或不存在的资源
        String[] paths = {"/drone", "/world", "/composed"};
        for (String path : paths) {
            try {
                template.removeAll(path);
                System.out.println("  ✓ Cleared path: " + path);
            } catch (org.eclipse.emf.cdo.view.CDOViewSet.CDOViewSetException e) {
                // CDO 脏资源错误：资源有未提交的更改，忽略（可能是前次运行遗留）
                System.out.println("  ⚠ Skipped dirty resource: " + path + " (will be overwritten)");
            } catch (Exception e) {
                // 其他错误（如资源不存在）：忽略
                System.out.println("  ! Error cleaning " + path + " (ignorable): " + e.getClass().getSimpleName());
            }
        }
        
        System.out.println("✓ CDO database ready");
    }
    
    /**
     * // 步骤4：注册元模型到CDO（为步骤6插入实例模型到CDO做准备）
     */
    private void registerMetaModelToCDO() throws Exception {
        //System.out.println("注册元模型到CDO...");
        
        DynamicSignature signature = sig();
        EPackage metaModel = createOrGetBigraphMetaModel(signature); //根据签名创建或获取对应的 EMF 元模型

        EPackage.Registry.INSTANCE.put(metaModel.getNsURI(), metaModel); //将元模型注册到 EMF 的 EPackage 注册表中，让 EMF 框架知道这个元模型
        CDOPackageRegistry.INSTANCE.put(metaModel.getNsURI(), metaModel); //将元模型注册到 CDO 的 CDO 包注册表中，让 CDO 框架知道这个元模型
        template.getCDOPackageRegistry().put(metaModel.getNsURI(), metaModel); //将元模型注册到 Spring Data CDO 的 CDO 包注册表中，让这个特定的 CDO 模板知道元模型
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
            
            // 确保world签名已加载
            if (serviceWorldSignature == null) {
                throw new IllegalStateException("World signature not loaded yet!");
            }
            
            // 使用 BigraphUtil.mergeSignatures 方法合并（与 DroneLandingSystem4x5.java 相同）
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
     * 创建Drone签名
     * 注意：使用 .add() 而不是 .addControl()，因为版本差异
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
     * 从本地获取World Model签名
     */
    private DynamicSignature fetchWorldSignatureFromService() throws Exception {
        if (serviceWorldSignature != null) {
            return serviceWorldSignature;
        }
        
        System.out.println("\n==================Local World Signature======================");
        
        // 从本地签名文件加载
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
        //System.out.println("\n===========获取World Model实例=============================");
        
        // 关键：使用合并签名创建bigraph元模型
        DynamicSignature combinedSig = sig();
        EPackage metaModel = createOrGetBigraphMetaModel(combinedSig);
        
        //System.out.println("  签名控制数: " + combinedSig.getControls().size());
        
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

        // 从JSON响应中提取XML内容
        String xmlContent = extractXmlFromJsonResponse(response.body());
        
        // 使用合并签名的元模型来反序列化实例模型
        try (ByteArrayInputStream inputStream = new ByteArrayInputStream(xmlContent.getBytes(StandardCharsets.UTF_8))) {
            // 使用合并签名的元模型加载实例
            List<EObject> worldObjects = BigraphFileModelManagement.Load.bigraphInstanceModel(metaModel, inputStream);
            
            // 使用合并签名转换为Bigraph
            PureBigraph bigraph = BigraphUtil.toBigraph(metaModel, worldObjects.get(0), combinedSig);
            //System.out.println("  反序列化成功，Site数量: " + bigraph.getSites().size());
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
     * 从JSON响应中提取XML内容
     */
    private String extractXmlFromJsonResponse(String responseBody) throws Exception {
        String trimmedBody = responseBody.trim();
        
        try {
            // 检查是否为JSON格式
            if (trimmedBody.startsWith("{")) {
                JsonNode jsonNode = objectMapper.readTree(trimmedBody);
                
                // 检查是否有content字段
                if (jsonNode.has("content")) {
                    String content = jsonNode.get("content").asText();
                    
                    // 检查mimeType
                    String mimeType = jsonNode.has("mimeType") ? jsonNode.get("mimeType").asText() : "";
                    
                    // 如果mimeType表明是XML，直接返回content
                    if (mimeType.contains("xml")) {
                        //System.out.println("从JSON包装中提取XML内容");
                        return content;
                    }
                    
                    // 如果没有明确的xml mimeType，尝试检测content是否为XML
                    if (content.trim().startsWith("<?xml") || content.trim().startsWith("<")) {
                        //System.out.println("内容看起来是XML，直接使用");
                        return content;
                    }
                }
                
                throw new IllegalStateException("JSON响应不包含有效的XML内容");
            }
            
            // 如果不是JSON，假设是纯XML
            if (trimmedBody.startsWith("<?xml") || trimmedBody.startsWith("<")) {
                System.out.println("Response appears to be pure XML");
                return trimmedBody;
            }
            
            throw new IllegalStateException("响应既不是有效的JSON也不是XML。前100个字符: " + 
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

    //用于初始化第一次放置drone model
    private PureBigraph droneModel(int siteCount) throws InvalidConnectionException, TypeNotExistsException {
        if (siteCount <= 0) {
            return pureBuilder(sig()).create();
        }

        List<Bigraph<DynamicSignature>> placements = new ArrayList<>();
        for (int i = 0; i < siteCount; i++) {
            // 检查是否是障碍物栅格
            if (obstacleGrids.contains(i)) {
                placements.add(buildObstacleCell());
            } else {
                placements.add(emptyOccupiedCell()); 
            }
        }

        int dronesToPlace = Math.min(configuredDroneCount, siteCount);
        for (int idx = 0; idx < dronesToPlace; idx++) {
            // 跳过障碍物栅格，不在障碍物位置放置无人机
            if (obstacleGrids.contains(idx)) {
                continue;
            }
            
            String droneId = "D" + idx;
            // 获取无人机的实际状态
            String droneStatus = "Landed";  // 默认状态
            DroneStatus status = droneStatuses.get(droneId);
            if (status != null) {
                if (status.landingRuleApplied) {
                    droneStatus = "Landed";  // 降落规则已应用，状态为 Landed
                } else if (status.takeoffRuleApplied) {
                    droneStatus = "flying";  // 起飞规则已应用，状态为 flying
                }
            }
            placements.set(idx, buildDrone(droneId, droneStatus, "OccupiedBy"));
        }

        Bigraph<DynamicSignature> result = placements.stream()
                .reduce(pureLinkings(sig()).identity_e(), accumulator::apply); //pureLinkings(sig()).identity_e()是一个空bigraph
        return (PureBigraph) result;
    }

    private PureBigraph buildDrone(String id, String status, String nodeType) throws InvalidConnectionException, TypeNotExistsException {
        PureBigraphBuilder<DynamicSignature> builder = pureBuilder(sig());
        String normalizedId = id.toLowerCase(Locale.ROOT);
        
        // 获取内存中无人机的实际电池和通信状态，用于构建每一轮的drone model
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
     * 构建带障碍物的OccupiedBy节点
     * OccupiedBy节点下包含ObstacleOrWeather节点，表示该栅格被障碍物占用
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

    // 用于合并 Bigraph 列表的累加器，parallelProduct：将两个Bigraph并排合并
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

    /** 通用电池状态转换规则 */
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

    /** 通用通信状态转换规则 */
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
     * 创建方向性移动规则（通用方法）
     * @param id 无人机ID
     * @param routeType Route类型（ForwardRoute, BackRoute, LeftRoute, RightRoute）
     * @return 移动规则
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
     * 发送HTTP POST请求到无人机控制服务
     * @param port 端口号
     * @param endpoint API端点（如 "/activate_idle", "/begin_takeoff"）
     * @param successMessage 成功消息
     * @param errorPrefix 错误消息前缀
     * @return 是否成功
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
     * 发送无人机控制指令：激活 idle 状态
     */
    private boolean activateIdle(String droneId, int port) {
        return sendDroneControlRequest(droneId, port, "/activate_idle", "Idle state has been activated.", "activate idle");
    }
    
    /**
     * 发送无人机控制指令：开始起飞
     */
    private boolean beginTakeoff(String droneId, int port) {
        return sendDroneControlRequest(droneId, port, "/begin_takeoff", "Takeoff command has been transmitted.", "takeoff");
    }
    
    /**
     * 发送无人机导航指令：移动到指定位置
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
     * 发送无人机控制指令：开始降落
     */
    private boolean beginLanding(String droneId, int port) {
        return sendDroneControlRequest(droneId, port, "/begin_landing", "Landing command has been transmitted.", "landing");
    }
    
    /**
     * 对所有无人机执行起飞流程：规则匹配 -> 发送起飞指令
     * 【修复】使用最新的 composite 字段而不是传入参数，确保电池/通信规则应用后能获取最新状态
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
            // 在起飞前先应用一次电池和通信规则，确保Bigraph中的状态是最新的
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
                
                // 输出当前电池状态
                DroneStatus status = droneStatuses.get(droneId);
                if (status != null) {
                    System.out.println("  Current status - Battery: " + status.batteryLevel + 
                            " (" + String.format("%.2f", status.batteryVoltage) + "V), Comm: " + status.communicationStatus);
                }
                
                // 创建起飞规则
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
                    
                    TimeUnit.MILLISECONDS.sleep(500);  // 等待状态稳定（不计入上述时延）
                    
                    if (!beginTakeoff(droneId, port)) {
                        System.err.println("  !! Unable to send takeoff command, skipping " + droneId);
                        continue;
                    }
                    
                    // 电池保护期，保证在起飞前五秒电池状态不发生变化，在发送起飞命令时立即设置 takeoffTime
                    if (status != null) {
                        status.takeoffTime = System.currentTimeMillis();
                        status.takeoffCommandSent = true;  // 标记已发送起飞命令
                    }
                    
                    System.out.println("  ✓ " + droneId + " takeoff sequence initiated");
                } else {
                    // 输出详细的规则不匹配原因
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
     * 集体升高：起飞后把所有无人机统一抬升到 {@code collectiveAscentAltitude} 再进入导航循环。
     * 通过 collective.ascent.enabled 开关，collective.ascent.altitude 设定初始工作高度。
     * 步骤：1) 等待所有无人机 hasTakenOff；2) 保持各自水平位置逐机发送升高指令；3) 稳定等待。
     * 说明：水平移动保持当前高度、垂直移动按 layerHeight 变化，因此抬升后的基准高度会在后续导航中自然保持。
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

        // 1) 等待所有（非障碍位）无人机完成起飞
        long waitStart = System.currentTimeMillis();
        while (true) {
            int taken = 0, active = 0;
            for (int i = 0; i < configuredDroneCount; i++) {
                if (obstacleGrids.contains(i)) {
                    continue; // 障碍位不放无人机
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

        // 2) 逐机发送升高指令：保持当前 x,y，只抬升高度
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

        // 3) 稳定等待，让 status.z 收敛到目标高度后再进入导航
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
     * 计算一次导航移动的目标高度 Z。
     * <p>绝对模式（navigation.absolute-altitude.enabled=true）：目标高度 = base-altitude + 目标格所在层 * layerHeight，
     * 与当前物理高度无关。水平和垂直移动都对齐到目标层的绝对高度，因此欠冲/漂移会在下一次移动时被自动校正，
     * 同一层的所有无人机始终收敛到相同高度，永不越过层边界。
     * <p>相对模式（默认）：保留原逻辑——垂直移动在当前高度上 ±layerHeight，水平移动保持当前高度。
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

        // 相对模式（原逻辑）
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
     * 斜向移动的额外安全检查：
     * 对于 FORWARD_LEFT / FORWARD_RIGHT / BACK_LEFT / BACK_RIGHT，
     * 需要同时检查经过路径上的两个正交相邻格子（例如：前、左）是否被占用或预订。
     * 如果任意一个格子被其他无人机占用/预订，或是障碍栅格，则本次斜向移动视为不安全。
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
        
        // 根据斜向方向，计算路径上需要检查的两个正交格子
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
            // 忽略与目标格相同的索引（目标格会在Bigraph规则 + 预订逻辑中单独检查）
            if (idx == next.gridIndex) {
                continue;
            }
            
            // 只根据其他无人机的当前位置进行动态碰撞检查（障碍物和预订在这里不考虑）
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
                        // 策略1: 对方目标是我当前格子 -> 我在让路离开，放行
                        if (otherTarget != null && otherTarget.gridIndex == current.gridIndex) {
                            continue;
                        }
                        
                        // 策略2: 互锁死锁检测 — 双方都在等待时，数字ID小的优先通过
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
                        
                        // 策略3: 连续等待超过阈值，强制放行避免永久死锁
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
     * 辅助方法：在合法范围内将 (layer, x, y) 转换为 gridIndex 加入检查列表。
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
        final long WAIT_TIMEOUT = 8000;  // 等待超时时间（毫秒）
        
        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();
            
            // 必须已起飞且起飞规则已应用
            if (!status.hasTakenOff || !status.takeoffRuleApplied) {
                continue;
            }
            
            // 如果已到达目标，释放所有预订并跳过
            if (status.reachedDestination) {
                releaseAllGrids(droneId);
                continue;
            }
            
            try {
                // 【修复】在导航前检查是否需要触发应急响应
                // 如果电池已经是 Low/Empty 但还没有触发应急响应，则跳过本次导航
                // 等待状态监控线程在下一秒内触发应急响应
                boolean needsEmergency = ("Low".equals(status.batteryLevel) || "Empty".equals(status.batteryLevel) 
                        || "Bad".equals(status.communicationStatus)) && !status.isInEmergency;
                if (needsEmergency) {
                    // 跳过本次导航，等待应急响应触发
                    continue;
                }
                
                // 【应急响应】选择目标点：应急状态下使用emergencyTarget，否则使用原始目标
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
                
                // 获取当前位置
                DronePosition currentPos = dronePositions.get(droneId);
                if (currentPos == null || currentPos.gridIndex < 0) {
                    continue;
                }
                
                // 检查是否正在移动中（已发送指令但ROS2位置还未更新）
                if (status.isMoving && status.movingToGrid != null) {
                    // 检查是否已到达目标grid
                    if (currentPos.gridIndex == status.movingToGrid) {
                        // 已到达，重置移动状态
                        System.out.println("  ✓ " + droneId + " arrived at Grid[" + status.movingToGrid + "]");
                        
                        // 释放所有旧的预订（除了当前位置）
                        releaseAllGrids(droneId);
                        
                        // 预订当前位置（防止其他无人机占用）
                        tryReserveGrid(droneId, currentPos.gridIndex);
                        status.reservedGrid = currentPos.gridIndex;
                        
                        status.isMoving = false;
                        status.movingToGrid = null;
                        status.avoidGrids.clear();
                        status.consecutiveWaitCycles = 0;
                    } else {
                        // 移动超时：若长时间未到达 movingToGrid（位置漂移或指令不同步），则放弃等待并从当前位置重新规划
                        if (currentTime - status.lastMoveTime > MOVE_TIMEOUT_MS) {
                            System.out.println("  ⏱ " + droneId + " move timeout: expected Grid[" + status.movingToGrid + "], current Grid[" + currentPos.gridIndex + "], re-planning from current position");
                            releaseGrid(droneId, status.movingToGrid);
                            status.reservedGrid = null;
                            status.isMoving = false;
                            status.movingToGrid = null;
                            // 不 continue，继续往下执行，从 currentPos 重新规划
                        } else {
                            // 还在移动中，跳过本次导航控制
                            continue;
                        }
                    }
                }
                
                // 检查是否已到达最终目标
                if (currentPos.gridIndex == target.gridIndex) {
                    // 【应急响应】根据应急类型处理到达事件
                    if (status.isInEmergency) {
                        if ("EMPTY_BATTERY".equals(status.emergencyType) || "LOW_BATTERY_BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n🚨 [EMERGENCY LANDING] " + droneId + " reached emergency landing point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.emergencyLanded = true;
                            // 紧急降落后不再移动
                        } else if ("LOW_BATTERY".equals(status.emergencyType)) {
                            System.out.println("\n🔋 [RECHARGE POINT] " + droneId + " reached recharge point Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;
                            // 等待充电和起飞
                        } else if ("BAD_COMM".equals(status.emergencyType)) {
                            System.out.println("\n📡 [HOME REACHED] " + droneId + " reached home Grid[" + target.gridIndex + "]");
                            status.reachedDestination = true;
                            status.waitingForRecharge = true;
                            // 等待通信恢复
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
                
                // 限制移动频率（避免过于频繁）
                if (currentTime - status.lastMoveTime < 2000) { // 2秒移动一次
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
                
                // 预订成功！
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
                
                // 计算导航目标的Z坐标（绝对模式=按目标层绝对高度，自我校正；相对模式=当前高度±layerHeight）
                double targetZ = computeTargetZ(direction, status, nextPoint);
                
                if (navigateTo(droneId, port, nextPoint.x, nextPoint.y, targetZ)) {
                    status.lastMoveTime = currentTime;
                    
                    // 设置移动状态（正在移动中，等待ROS2位置更新）
                    status.isMoving = true;
                    status.movingToGrid = nextPoint.gridIndex;
                    
                    // 释放当前位置的预订（如果有）
                    if (currentPos.gridIndex != nextPoint.gridIndex) {
                        releaseGrid(droneId, currentPos.gridIndex);
                    }
                } else {
                    System.err.println(" !! Navigation command failed");
                    // 释放预订
                    releaseGrid(droneId, nextPoint.gridIndex);
                    status.reservedGrid = null;
                    status.isMoving = false;
                    status.movingToGrid = null;
                }
                
            } catch (ClassCastException e) {
                // 框架 PureBigraph.getPorts() 假定 REFERENCE_PORT 为 EList，EMF/CDO 有时返回数组导致匹配失败
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

                // Baseline: 无任何防护——不做斜向扫掠检查(isDiagonalPathClear)、不做 bigraph 门校验、不预订。
                // 直接无条件执行 A* 的下一步，因此不会因等待而死锁；冲突(同步汇聚同一格)会真实发生并被统计。
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
                // 存储被阻塞格子，下次 planNextStep 会绕开
                status.avoidGrids.add(blockedPoint.gridIndex);
                status.avoidGridsSetTime = currentTime;
            } else {
                // 找不到绕行路径，也记录被阻塞格子，并重置计时器重新等待
                status.avoidGrids.add(blockedPoint.gridIndex);
                status.avoidGridsSetTime = currentTime;
                status.waitingStartTime = currentTime;
            }
        }
    }
    
    /** 应用Bigraph规则并持久化
     * @param droneId 无人机ID
     * @param rule 要应用的规则
     * @param logPrefix 日志前缀
     * @return 是否成功应用规则
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
                // 框架 PureBigraph.getPorts() 假定 REFERENCE_PORT 为 EList，EMF/CDO 在某些情况下返回数组导致 [Ljava.lang.Object; cannot be cast to List
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
     * 检查并应用起飞规则（当无人机实际起飞后）
     * 注意：此方法会修改 composite 字段和 cdoIdComposed
     */
    private boolean checkAndApplyTakeoffRules() {
        boolean anyRuleApplied = false;
        
        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();
            
            // 如果无人机已经起飞但规则尚未应用
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
                        status.status = "flying"; //保存新状态在内存中，重新构建world model时采用
                        status.takeoffRuleApplied = true;
                        anyRuleApplied = true;
                    } else {
                        // 【修复】起飞规则不匹配时，输出原因并仍然更新内存状态
                        // 这是因为物理上无人机已经起飞，需要保持状态一致
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
                        
                        // 更新内存状态，让 droneModelFromRosPositions 在下次更新时正确设置 Bigraph 状态
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
     * 检查并应用降落规则（当无人机到达目标并实际降落后）
     * 注意：此方法会修改 composite 字段和 cdoIdComposed
     */
    private boolean checkAndApplyLandingRules() {
        boolean anyRuleApplied = false;
        long currentTime = System.currentTimeMillis();
        
        // 独立处理每架无人机的降落逻辑
        for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
            String droneId = entry.getKey();
            DroneStatus status = entry.getValue();
            
            // 步骤1：检查是否到达目标点 -> 立即发送降落指令（不等待其他无人机）
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
                continue; // 发送降落指令后，继续处理下一架无人机
            }
            
            // 步骤2：检查是否已降落 -> 应用降落规则（独立处理）
            if (status.landingCommandSent && !status.landingRuleApplied) {
                long timeSinceLandingCommand = currentTime - status.landingCommandTime;
                
                if (timeSinceLandingCommand >= 2000 && status.hasLanded) { // 2秒后且已降落
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
     * 持续监测电池和通信状态，并应用相应规则
     * 独立线程运行，与位置更新和导航控制并行
     */
    // ==================== 故障注入 (Fault Injection) ====================
    // 通过一个内置的轻量 HTTP 服务（JDK 自带，无需额外依赖）在运行时随时注入故障。
    // 注入的是"合成传感器读数"，其余流程（Bigraph 规则、应急响应）完全走真实代码路径。
    //
    // 用法示例（另开一个终端 curl，或浏览器直接访问）：
    //   让 D1 电池 Low：       curl "http://localhost:8090/inject/battery?drone=D1&level=Low"
    //   让 D1 电池 Empty：     curl "http://localhost:8090/inject/battery?drone=D1&level=Empty"
    //   让 D2 通信 Bad：       curl "http://localhost:8090/inject/comm?drone=D2&state=Bad"
    //   指定原始电压：         curl "http://localhost:8090/inject/battery?drone=D1&value=3.12"
    //   清除注入(恢复真实值)： curl "http://localhost:8090/inject/battery?drone=D1&level=clear"
    //                         curl "http://localhost:8090/inject/comm?drone=D2&state=clear"
    //   查看当前注入状态：     curl "http://localhost:8090/inject/status"
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
            server.setExecutor(null); // 使用默认 executor
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

    /** 处理 /inject/battery 与 /inject/comm；battery=true 表示电池注入，否则通信注入 */
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

    /** 设置电池注入。返回描述字符串；非法值返回 null。 */
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
        st.batteryVoltage = v; // 立即生效，不必等下一条 ROS 消息
        return "battery injected -> " + String.format(Locale.ROOT, "%.2fV", v);
    }

    /** 设置通信注入。返回描述字符串；非法值返回 null。 */
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
        st.rssi = r; // 立即生效
        return "comm injected -> RSSI " + r;
    }

    /** 处理 /inject/status：返回所有无人机当前的注入与状态 */
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
                TimeUnit.MILLISECONDS.sleep(1000);  // 每秒检查一次
                
                if (!rosUpdateEnabled) {
                    continue;
                }
                
                // 检查每架无人机的状态并应用规则
                for (Map.Entry<String, DroneStatus> entry : droneStatuses.entrySet()) {
                    String droneId = entry.getKey();
                    DroneStatus status = entry.getValue();
                    
                    try {
                        // 检查电池状态
                        checkAndApplyBatteryRule(droneId, status);
                        
                        // 检查通信状态
                        checkAndApplyCommunicationRule(droneId, status);
                        
                        // 检查并触发应急响应
                        checkAndTriggerEmergencyResponse(droneId, status);

                        // 检查应急状态恢复（通信恢复）
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
    
    /** 检查并应用电池状态转换规则 */
    private void checkAndApplyBatteryRule(String droneId, DroneStatus status) {
        long timeSinceTakeoff = System.currentTimeMillis() - status.takeoffTime;
        boolean inTakeoffProtection = (timeSinceTakeoff < TAKEOFF_PROTECTION_MS) && 
                status.takeoffCommandSent && !status.takeoffRuleApplied;
        
        // 连续应用转换，直到电池状态与当前电压一致。
        // 关键：注入 Empty(如 3.0V) 时，一个监测周期内 Normal→Low→Empty 会全部完成，
        // 使随后的应急检查直接看到 Empty 并触发就地降落，而不会中途停在 Low 触发返航。
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
    
    /** 检查并应用通信状态转换规则 */
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
    
    /** 检查并触发应急响应 */
    private void checkAndTriggerEmergencyResponse(String droneId, DroneStatus status) {
        if (!status.hasTakenOff) return;

        boolean lowBattery = "Low".equals(status.batteryLevel);
        boolean emptyBattery = "Empty".equals(status.batteryLevel);
        boolean badComm = "Bad".equals(status.communicationStatus);

        // 已在应急中：只允许"升级"到更高优先级——电池耗尽(Empty)必须就地降落，
        // 覆盖正在进行的 Low 返航 / BAD_COMM 返航（对应论文中"就地降落优先于长距离返航"）。
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
    
    /** 触发紧急降落 */
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
    
    /** 触发返回充电 */
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
            // 有充电站且距离更近，去充电站
            status.emergencyTarget = selectedStation;
            System.out.println("  → Charging Station: " + selectedStation);
        } else if (startPoint != null) {
            // 没有可用充电站或充电站更远，返航到起点
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
            // 起点也不存在（异常情况），紧急降落
            System.err.println("  !! No start point available, triggering emergency landing");
            triggerEmergencyLanding(droneId, status, "LOW_BATTERY");
            return;
        }
    }
    
    /** 触发返回起点 */
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
    
    
    /** 检查应急状态恢复（通信恢复日志） */
    private void checkEmergencyRecovery(String droneId, DroneStatus status) {
        if ("BAD_COMM".equals(status.emergencyType) && "Normal".equals(status.communicationStatus)) {
            System.out.println("📡 [INFO] " + droneId + " Comm recovered, continuing home");
        }
    }
    
    // ========================================
    // ROS2 订阅和位置管理
    // ========================================
    
    /**
     * 订阅 ROS2 话题获取无人机位置
     * @param host ROS bridge 主机地址
     * @param topic 话题名称
     * @param type 消息类型
     * @param handler 消息处理器
     */
    // 共享的 rosbridge 连接：所有 topic 订阅复用同一个 WebSocket，避免每个 topic 各建一条连接、
    // 在多机/多 topic 时出现 "Could not create WebSocket: Connection failed"（尤其 sim 模式）。
    private Ros sharedRos;

    /** 确保共享 rosbridge 连接已建立；成功返回 true。 */
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
     * 初始化所有无人机的 ROS2 订阅
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
        
        // 初始化所有无人机位置和状态为默认值
        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            // 初始化为默认位置（第1层左下角，网格索引 i）
            dronePositions.put(droneId, new DronePosition(droneId, 0, 0, 0, i));
            // 初始化状态
            droneStatuses.put(droneId, new DroneStatus(droneId));
        }
        System.out.println("✓ Initialized default positions and status for " + configuredDroneCount + " drones");
        
        // 位置订阅：根据是否为仿真模式，选择单 topic 或多 topic
        if (rosSimMode) {
            // 仿真模式：从 /cf_positions_path (nav_msgs/Path) 获取，根据 poses[].header.frame_id 识别 cf231/cf232/...
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
                        // frame_id 如 "cf231", "cf232", "cf233" -> 231,232,233 -> D0, D1, D2
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
            // 实际模式：每架无人机订阅各自的 /cfXXX/pose
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
        
        // 状态话题（电池电压和RSSI）始终按每架无人机单独订阅
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
                            // 若存在故障注入覆盖，则忽略真实读数，改用注入值（清除注入后自动恢复真实值）
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
     * 将世界坐标转换为3D网格索引
     * 
     * 网格布局说明（3D版本）：
     * - 网格中心点坐标：(gridOriginX + xIndex * gridStepX, gridOriginY + yIndex * gridStepY, gridOriginZ + layerIndex * layerHeight)
     * - 网格边界：中心点 ± gridStep/2 (XY平面), ± layerHeight/2 (Z轴)
     * - 层级判断：0 <= z < 0.3 第1层, 0.3 <= z < 0.6 第2层, 以此类推
     * - 公式：index = layerIndex * (bigridCols * bigridRows) + xIndex * bigridRows + yIndex
     * 
     * 示例（5x5x3）：
     * - 第1层（layer 0）：Grid[0-24]，右下角Grid[0]到左上角Grid[24]
     * - 第2层（layer 1）：Grid[25-49]，右下角Grid[25]到左上角Grid[49]
     * - 第3层（layer 2）：Grid[50-74]，右下角Grid[50]到左上角Grid[74]
     * 
     * @param x 世界坐标 X
     * @param y 世界坐标 Y
     * @param z 世界坐标 Z
     * @return 网格索引（0-based），如果超出范围返回 -1
     */
    private int coordinateToGridIndex(double x, double y, double z) {
        // 容差值，用于处理浮点数精度问题
        final double EPSILON = 1e-6;
        
        // 处理 -0.0 的情况，将其视为 0.0
        if (Math.abs(x) < EPSILON) x = 0.0;
        if (Math.abs(y) < EPSILON) y = 0.0;
        if (Math.abs(z) < EPSILON) z = 0.0;
        
        // 计算相对于网格原点的偏移
        double relX = x - gridOriginX;
        double relY = y - gridOriginY;
        double relZ = z - gridOriginZ;
        
        // 使用四舍五入计算XY索引
        int xIndex = (int) Math.round(relX / gridStepX);
        int yIndex = (int) Math.round(relY / gridStepY);
        
        // 计算层索引（Z轴）：使用floor来确定在哪一层
        // 0 <= z < 0.3 -> layer 0
        // 0.3 <= z < 0.6 -> layer 1
        // 0.6 <= z < 0.9 -> layer 2
        int layerIndex = (int) Math.floor(relZ / layerHeight);

        // 检查是否在有效范围内
        if (xIndex < 0 || xIndex >= bigridCols || 
            yIndex < 0 || yIndex >= bigridRows || 
            layerIndex < 0 || layerIndex >= bigridLayers) {
            return -1; // 超出网格范围
        }
        
        // 计算3D线性索引
        // 每层有 bigridCols * bigridRows 个格子
        // 同层内：X列优先，即同一X列的Y依次排列
        int index = layerIndex * (bigridCols * bigridRows) + xIndex * bigridRows + yIndex;
        
        return index;
    }
    
    // /**
    //  * 将世界坐标转换为网格索引（2D版本，用于兼容性）
    //  * @deprecated 请使用3D版本 coordinateToGridIndex(x, y, z)
    //  */
    // @Deprecated
    // private int coordinateToGridIndex(double x, double y) {
    //     return coordinateToGridIndex(x, y, 0.0);
    // }
    
    // ========================================
    // 路径规划和导航
    // ========================================
    
    /**
     * 解析目标点配置，并初始化充电站、障碍物和无人机起始位置
     */
    private void parseDroneTargets() {
        System.out.println("\n========================================");
        System.out.println("Parsing drone target points, charging station, and obstacles");
        System.out.println("========================================");
        
        // 初始化障碍物栅格（支持多个障碍物，用逗号分隔）
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
        
        // 初始化充电站位置（支持多个充电站，用逗号分隔）
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
                    
                    // 目标点使用第1层的Z坐标进行网格索引计算（假设目标在第1层）
                    // 但实际飞行高度由无人机当前高度决定，不需要改变
                    double targetZ = gridOriginZ;  // 第1层
                    int gridIndex = coordinateToGridIndex(x, y, targetZ);
                    
                    String droneId = "D" + i;
                    // 目标点的Z坐标设为第1层的高度（仅用于网格索引，不影响实际飞行高度）
                    droneTargets.put(droneId, new GridPoint(x, y, targetZ, gridIndex));
                    System.out.println("  " + droneId + " target: (" + x + ", " + y + ") -> Grid[" + gridIndex + "] (Layer 1)");
                    
                    // 【修复】记录无人机的起始位置（从ROS2实际位置获取，而不是循环索引）
                    DroneStatus status = droneStatuses.get(droneId);
                    DronePosition currentPos = dronePositions.get(droneId);
                    if (status != null) {
                        // 使用ROS2报告的实际位置作为起始位置
                        if (currentPos != null && currentPos.gridIndex >= 0) {
                            status.startGridIndex = currentPos.gridIndex;
                            System.out.println("  " + droneId + " start position: Grid[" + currentPos.gridIndex + "] (from ROS2)");
                        } else {
                            // 如果还没有ROS2数据，使用默认位置
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
     * 从3D网格索引计算中心坐标
     */
    private GridPoint gridIndexToPoint(int gridIndex) {
        int totalGridsPerLayer = bigridCols * bigridRows;
        if (gridIndex < 0 || gridIndex >= totalGridsPerLayer * bigridLayers) {
            return null;
        }
        
        // 计算层索引
        int layerIndex = gridIndex / totalGridsPerLayer;
        int indexInLayer = gridIndex % totalGridsPerLayer;
        
        // 计算XY索引
        int xIndex = indexInLayer / bigridRows;
        int yIndex = indexInLayer % bigridRows;
        
        // 计算世界坐标
        double x = gridOriginX + xIndex * gridStepX;
        double y = gridOriginY + yIndex * gridStepY;
        double z = gridOriginZ + layerIndex * layerHeight;
        
        return new GridPoint(x, y, z, gridIndex);
    }
    
    /**
     * 获取3D网格的邻居（10个方向：8个水平+2个垂直）
     */
    private List<GridPoint> getNeighbors(int gridIndex) {
        List<GridPoint> neighbors = new ArrayList<>();
        
        int totalGridsPerLayer = bigridCols * bigridRows;
        int layerIndex = gridIndex / totalGridsPerLayer;
        int indexInLayer = gridIndex % totalGridsPerLayer;
        
        int xIndex = indexInLayer / bigridRows;
        int yIndex = indexInLayer % bigridRows;
        
        // 10个方向：8个水平方向 + 2个垂直方向
        // 水平方向（同一层）：左、右、下、上、左下、左上、右下、右上
        int[][] horizontalDirections = {
            {-1, 0, 0},  // 左
            {1, 0, 0},   // 右
            {0, -1, 0},  // 下
            {0, 1, 0},   // 上
            {-1, -1, 0}, // 左下
            {-1, 1, 0},  // 左上
            {1, -1, 0},  // 右下
            {1, 1, 0}    // 右上
        };
        
        // 添加水平邻居
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
        
        // 添加垂直邻居（上层和下层）
        // 向上（layer + 1）
        if (layerIndex + 1 < bigridLayers) {
            int upIndex = (layerIndex + 1) * totalGridsPerLayer + xIndex * bigridRows + yIndex;
            GridPoint upPoint = gridIndexToPoint(upIndex);
            if (upPoint != null) {
                neighbors.add(upPoint);
            }
        }
        
        // 向下（layer - 1）
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
     * 计算两点之间的3D欧几里得距离（启发式函数）
     */
    private double heuristic(GridPoint a, GridPoint b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2) + Math.pow(a.z - b.z, 2));
    }
    
    /**
     * 动态A*规划：每次移动前重新规划最短路径的下一步
     * @param droneId 无人机ID
     * @param start 起点
     * @param goal 终点
     * @return 下一步要移动到的grid，如果无法规划则返回null
     */
    private GridPoint planNextStep(String droneId, GridPoint start, GridPoint goal) {
        if (start.gridIndex == goal.gridIndex) {
            return null;  // 已到达目标
        }
        
        // 获取当前障碍物（其他无人机的位置）
        Set<Integer> occupiedGrids = getOccupiedGrids(droneId);
        
        // 加入临时规避格子（绕行时记录的被阻塞格子，30秒后过期）
        DroneStatus planStatus = droneStatuses.get(droneId);
        if (planStatus != null && !planStatus.avoidGrids.isEmpty()) {
            if (System.currentTimeMillis() - planStatus.avoidGridsSetTime < 30000) {
                occupiedGrids.addAll(planStatus.avoidGrids);
            } else {
                planStatus.avoidGrids.clear();
            }
        }
        
        // 使用A*规划完整路径
        List<GridPoint> fullPath = planPath(start, goal, occupiedGrids);

        // 日志：输出完整路径（如果存在）
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
            return null;  // 无法规划路径
        }
        
        // 返回路径的下一步（索引1，索引0是起点）
        return fullPath.get(1);
    }
    
    /**
     * A*路径规划算法
     * @param start 起点
     * @param goal 终点
     * @param occupiedGrids 被占用的网格（需要避开）
     * @return 路径点列表（从起点到终点）
     */
    private List<GridPoint> planPath(GridPoint start, GridPoint goal, Set<Integer> occupiedGrids) {
        PriorityQueue<AStarNode> openSet = new PriorityQueue<>();
        Set<Integer> closedSet = new HashSet<>();
        Map<Integer, Double> gScores = new HashMap<>();
        
        openSet.add(new AStarNode(start, null, 0, heuristic(start, goal)));
        gScores.put(start.gridIndex, 0.0);
        
        while (!openSet.isEmpty()) {
            AStarNode current = openSet.poll();
            
            // 到达目标
            if (current.point.gridIndex == goal.gridIndex) {
                return reconstructPath(current);
            }
            
            closedSet.add(current.point.gridIndex);
            
            // 探索邻居
            for (GridPoint neighbor : getNeighbors(current.point.gridIndex)) {
                // 跳过已访问的节点
                if (closedSet.contains(neighbor.gridIndex)) {
                    continue;
                }
                
                // 跳过被占用的网格（但目标点除外）
                if (occupiedGrids.contains(neighbor.gridIndex) && neighbor.gridIndex != goal.gridIndex) {
                    continue;
                }
                
                // 计算代价（3D版本：对角线移动和垂直移动代价更高）
                boolean isDiagonalXY = Math.abs(neighbor.x - current.point.x) > 0.5 && 
                                      Math.abs(neighbor.y - current.point.y) > 0.5;
                boolean isVertical = Math.abs(neighbor.z - current.point.z) > 0.01;
                
                double moveCost;
                if (isDiagonalXY && !isVertical) {
                    moveCost = Math.sqrt(2);  // 水平对角线移动
                } else if (isVertical && !isDiagonalXY) {
                    moveCost = 1.5;  // 垂直移动（稍高代价，优先考虑水平移动）
                } else {
                    moveCost = 1.0;  // 直线移动
                }
                
                double tentativeGScore = current.gCost + moveCost;
                
                if (!gScores.containsKey(neighbor.gridIndex) || tentativeGScore < gScores.get(neighbor.gridIndex)) {
                    gScores.put(neighbor.gridIndex, tentativeGScore);
                    double hScore = heuristic(neighbor, goal);
                    openSet.add(new AStarNode(neighbor, current, tentativeGScore, hScore));
                }
            }
        }
        
        // 没有找到路径
        return new ArrayList<>();
    }
    
    /**
     * 重建路径
     */
    private List<GridPoint> reconstructPath(AStarNode node) {
        List<GridPoint> path = new ArrayList<>();
        AStarNode current = node;
        while (current != null) {
            path.add(0, current.point);  // 插入到开头
            current = current.parent;
        }
        return path;
    }
    
    /**
     * 获取所有其他无人机当前占用的网格（包括障碍物栅格 + 已被其他无人机预订的grid）
     */
    private Set<Integer> getOccupiedGrids(String excludeDroneId) {
        Set<Integer> occupied = new HashSet<>();
        
        // 添加其他无人机占用的网格（真实位置）
        for (Map.Entry<String, DronePosition> entry : dronePositions.entrySet()) {
            if (!entry.getKey().equals(excludeDroneId)) {
                DronePosition pos = entry.getValue();
                if (pos.gridIndex >= 0) {
                    occupied.add(pos.gridIndex);
                }
            }
        }
        
        // 添加障碍物栅格（障碍物永远被占用，无人机不能进入）
        occupied.addAll(obstacleGrids);
        
        // 添加被其他无人机预订的grid（视为临时障碍，当前无人机在A*规划时会绕开）
        for (Map.Entry<Integer, GridReservation> entry : gridReservations.entrySet()) {
            GridReservation reservation = entry.getValue();
            if (!reservation.droneId.equals(excludeDroneId)) {
                occupied.add(entry.getKey());
            }
        }
        
        return occupied;
    }
    
    // ========================================
    // Grid预订系统（同步锁机制）
    // ========================================
    
    /**
     * 尝试预订一个grid
     * @param droneId 无人机ID
     * @param gridIndex 要预订的grid索引
     * @return true 如果预订成功，false 如果grid已被其他无人机预订
     */
    private boolean tryReserveGrid(String droneId, int gridIndex) {
        GridReservation existingReservation = gridReservations.get(gridIndex);
        
        // 如果已经被预订
        if (existingReservation != null) {
            // 如果是自己预订的，返回成功
            if (existingReservation.droneId.equals(droneId)) {
                return true;
            }
            // 被其他无人机预订，返回失败
            return false;
        }
        
        // 尝试预订（使用ConcurrentHashMap的原子操作）
        GridReservation newReservation = new GridReservation(droneId, System.currentTimeMillis());
        GridReservation previous = gridReservations.putIfAbsent(gridIndex, newReservation);
        
        // 如果previous为null，说明预订成功
        if (previous == null) {
            return true;
        }
        
        // 如果previous不为null，检查是否是自己的预订
        return previous.droneId.equals(droneId);
    }
    
    /**
     * 释放grid预订
     * @param droneId 无人机ID
     * @param gridIndex 要释放的grid索引
     */
    private void releaseGrid(String droneId, int gridIndex) {
        GridReservation reservation = gridReservations.get(gridIndex);
        if (reservation != null && reservation.droneId.equals(droneId)) {
            gridReservations.remove(gridIndex);
        }
    }
    
    /**
     * 释放无人机的所有grid预订
     * @param droneId 无人机ID
     */
    private void releaseAllGrids(String droneId) {
        gridReservations.entrySet().removeIf(entry -> entry.getValue().droneId.equals(droneId));
    }
    
    /**
     * 检查grid是否被预订
     * @param gridIndex grid索引
     * @return 预订该grid的无人机ID，如果未被预订则返回null
     */
    private String getGridReservation(int gridIndex) {
        GridReservation reservation = gridReservations.get(gridIndex);
        return (reservation != null) ? reservation.droneId : null;
    }
    
    // ========================================
    // 碰撞检测
    // ========================================
    
    /**
     * 检测碰撞风险：如果两架或更多无人机在同一个网格
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

        // 去抖 + 飞行阶段过滤：只有当某格连续 violationDebounceFrames 帧都被"≥2架已起飞无人机"占用时，
        // 才计一次真违规。过滤两类假重叠：(a) 单帧/短闪跳（边界抖动、异步残影）；
        // (b) 起飞前地面/仿真初始化阶段的位置重叠（互斥是飞行阶段的性质，地面 spawn 重叠不算）。
        for (Integer gridIndex : currentViolatingGrids) {
            long airborneOccupants = violatingOccupants.get(gridIndex).stream()
                    .filter(id -> {
                        DroneStatus s = droneStatuses.get(id);
                        return s != null && s.hasTakenOff;
                    })
                    .count();
            if (airborneOccupants < 2) {
                // 该格的重叠里不足两架已起飞无人机（地面/起飞前），不计违规；重置其连续帧计数。
                // 注意：上面的 riskDetected/合并暂停仍按原始检测执行，保持保守。
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
        // 冲突结束的格子：重置其连续帧计数并清除"已计数"标记，使下一次持续冲突可重新计一次。
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
     * 根据 ROS2 实时位置更新无人机模型
     * @param siteCount 网格站点数量
     * @return 更新后的无人机模型
     */
    private PureBigraph droneModelFromRosPositions(int siteCount) throws InvalidConnectionException, TypeNotExistsException, IncompatibleSignatureException, IncompatibleInterfaceException {
        if (!rosUpdateEnabled || dronePositions.isEmpty()) {
            // 如果未启用 ROS2 或没有位置数据，使用默认放置策略
            return droneModel(siteCount);
        }
        
        // 检查碰撞风险
        if (collisionRiskDetected.get()) {
            System.err.println("⚠ Collision risk detected, using previous drone model");
            return dronePart; // 返回当前模型，不更新
        }
        
        // 创建空的站点列表（包含障碍物）
        List<Bigraph<DynamicSignature>> placements = new ArrayList<>();
        for (int i = 0; i < siteCount; i++) {
            // 检查是否是障碍物栅格
            if (obstacleGrids.contains(i)) {
                placements.add(buildObstacleCell());
            } else {
                placements.add(emptyOccupiedCell());
            }
        }
        
        // 根据 ROS2 位置放置无人机（使用实际状态）
        //遍历dronePositions Map中的所有值，pos 包含：droneId, x, y, gridIndex
        Set<Integer> placedCells = new HashSet<>();
        for (DronePosition pos : dronePositions.values()) {
            int placeIndex = pos.gridIndex;

            // 仅在起飞前(尚未 hasTakenOff)容忍定位漂移：grid=-1 时回退到起始格，
            // 使其留在模型中并允许正常起飞（takeoff.allow-out-of-grid 控制，可关闭）。
            // 起飞后不再回退——飞行途中若持续 grid=-1，保持模型与物理严格一致，暴露真实问题。
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
                // 跳过障碍物栅格，不在障碍物位置放置无人机
                if (obstacleGrids.contains(placeIndex)) {
                    continue;
                }

                // 获取无人机的实际状态
                String droneStatus = "Landed";  // 默认状态
                DroneStatus status = droneStatuses.get(pos.droneId);
                if (status != null) {
                    if (status.landingRuleApplied) {
                        droneStatus = "Landed";  // 降落规则已应用
                    } else if (status.takeoffRuleApplied) {
                        droneStatus = "flying";  // 起飞规则已应用
                    }
                }
                //在指定位置放置无人机
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
     * 位置落在栅格外(grid=-1)时的回退格：优先无人机起始格，其次按编号(D{idx}->cell idx)。
     * 返回 -1 表示无有效回退格。
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
            // droneId 非 "D<number>" 格式，无编号回退
        }
        return -1;
    }

    /**
     * 无人机位置信息（3D版本）
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
        // 故障注入覆盖值（null = 使用真实 ROS 读数）。非 null 时，ROS 上报值被忽略，改用注入值。
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
    
    // 3D网格点类
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
    
    // A*算法节点类
    private static class AStarNode implements Comparable<AStarNode> {
        final GridPoint point;
        final AStarNode parent;
        final double gCost;  // 从起点到当前点的实际代价
        final double hCost;  // 从当前点到终点的启发式代价
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
    
    // Grid预订信息类
    private static class GridReservation {
        final String droneId;           // 预订该grid的无人机ID
        final long reservationTime;     // 预订时间
        
        GridReservation(String droneId, long reservationTime) {
            this.droneId = droneId;
            this.reservationTime = reservationTime;
        }
    }
}
