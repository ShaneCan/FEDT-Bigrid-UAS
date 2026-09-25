package org.example;

import org.bigraphs.framework.core.AbstractEcoreSignature;
import org.bigraphs.framework.core.Bigraph;
import org.bigraphs.framework.core.BigraphFileModelManagement;
import org.bigraphs.framework.core.Control;
import org.bigraphs.framework.core.exceptions.IncompatibleSignatureException;
import org.bigraphs.framework.core.exceptions.InvalidConnectionException;
import org.bigraphs.framework.core.exceptions.builder.TypeNotExistsException;
import org.bigraphs.framework.core.exceptions.operations.IncompatibleInterfaceException;
import org.bigraphs.framework.core.impl.BigraphEntity;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.framework.core.impl.pure.PureBigraphBuilder;
import org.bigraphs.framework.core.impl.signature.DynamicControl;
import org.bigraphs.framework.core.impl.signature.DynamicSignature;
import org.bigraphs.framework.core.impl.signature.DynamicSignatureBuilder;
import org.bigraphs.framework.core.reactivesystem.InstantiationMap;
import org.bigraphs.framework.core.reactivesystem.ParametricReactionRule;
import org.bigraphs.framework.core.reactivesystem.BigraphMatch;
import org.bigraphs.framework.simulation.matching.AbstractBigraphMatcher;
import org.bigraphs.framework.simulation.matching.pure.PureReactiveSystem;
import org.bigraphs.framework.core.utils.BigraphUtil;
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

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;

import javax.json.JsonObject;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BinaryOperator;

import static org.bigraphs.framework.core.factory.BigraphFactory.createOrGetBigraphMetaModel;
import static org.bigraphs.framework.core.factory.BigraphFactory.createOrGetSignature;
import static org.bigraphs.framework.core.factory.BigraphFactory.ops;
import static org.bigraphs.framework.core.factory.BigraphFactory.pureBuilder;
import static org.bigraphs.framework.core.factory.BigraphFactory.pureLinkings;
import static org.bigraphs.framework.core.factory.BigraphFactory.pureSignatureBuilder;

// @SpringBootApplication  // 临时禁用，使用Application3D
@Import(value = {CDOServerConfig.class})
public class Application implements CommandLineRunner {

    private static final String WORLD_RESOURCE_BASE = "src/test/resources/models/diagonaldirectional/";
    private static final String WORLD_SIGNATURE_MM = WORLD_RESOURCE_BASE + "mm_sig_loc.ecore";
    private static final String WORLD_SIGNATURE_INSTANCE = WORLD_RESOURCE_BASE + "sig_loc.xmi";

    @Autowired
    protected CdoTemplate template;

    @Value("${bigrid.service.base-url:http://172.29.224.1:8080}")
    private String bigridServiceBaseUrl;

    @Value("${bigrid.service.rows:5}")
    private int bigridRows;

    @Value("${bigrid.service.cols:5}")
    private int bigridCols;

    @Value("${bigrid.service.format:xml}")
    private String bigridFormat;

    @Value("${bigrid.service.origin.x:-2.0}")
    private double gridOriginX;

    @Value("${bigrid.service.origin.y:-2.0}")
    private double gridOriginY;

    @Value("${bigrid.service.step.x:1.0}")
    private double gridStepX;

    @Value("${bigrid.service.step.y:1.0}")
    private double gridStepY;

    @Value("${drone.count:2}")
    private int configuredDroneCount;
    
    @Value("${ros.bridge.host:localhost}")
    private String rosBridgeHost;
    
    @Value("${ros.update.enabled:false}")
    private boolean rosUpdateEnabled;
    
    // Drone Control Configuration
    @Value("${drone.control.enabled:false}")
    private boolean droneControlEnabled;
    @Value("${drone.control.base-url:http://127.0.0.1}")
    private String droneControlBaseUrl;
    @Value("${drone.control.start-port:5000}")
    private int droneControlStartPort;
    
    // 无人机目标点配置 (格式: "x1,y1;x2,y2;...")
    @Value("${drone.targets:-1,1;0,1}")
    private String droneTargetsConfig;

    private final HttpClient httpClient = HttpClient.newBuilder()
            .connectTimeout(Duration.ofSeconds(5))
            .build();
    
    // 无人机目标点映射
    private final Map<String, GridPoint> droneTargets = new HashMap<>();

    private final ObjectMapper objectMapper = new ObjectMapper();
    
    // 无人机位置信息存储（线程安全）：droneId -> {x, y, gridIndex}
    private final Map<String, DronePosition> dronePositions = new ConcurrentHashMap<>();
    
    // 无人机状态信息存储（线程安全）：droneId -> {status, z, hasTakenOff}
    private final Map<String, DroneStatus> droneStatuses = new ConcurrentHashMap<>();
    
    // 碰撞风险标记
    private final AtomicBoolean collisionRiskDetected = new AtomicBoolean(false);
    
    // Grid预订系统：防止多个无人机同时移动到同一个grid
    private final Map<Integer, GridReservation> gridReservations = new ConcurrentHashMap<>();

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

    public static void main(String[] args) {
        BigraphBaseModelPackageImpl.init();
        SpringApplication.run(Application.class, args);
    }

    @Override
    public void run(String... args) throws Exception {
        TimeUnit.MILLISECONDS.sleep(2500);

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
        
        // 启动后台模型更新线程（持续快速更新）
        Thread modelUpdateThread = new Thread(this::continuousModelUpdate, "ModelUpdateThread");
        modelUpdateThread.setDaemon(false);
        modelUpdateThread.start();
        
        // 启动电池和通信状态监测线程（如果启用ROS2）
        if (rosUpdateEnabled) {
            Thread statusMonitorThread = new Thread(this::continuousStatusMonitoring, "StatusMonitorThread");
            statusMonitorThread.setDaemon(false);
            statusMonitorThread.start();
            System.out.println("✓ Battery and communication status monitoring thread started");
        }
        
        // 等待第一次模型更新完成
        System.out.println("Waiting for first model update to complete...");
        while (!firstUpdateCompleted.get()) {
            TimeUnit.MILLISECONDS.sleep(100);
        }
        System.out.println("✓ First model update completed\n");
        
        // 解析目标点配置
        parseDroneTargets();
        
        // 执行起飞序列：规则匹配并发送起飞指令
        if (droneControlEnabled) {
            performTakeoffSequence(composite);
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
            performNavigationControl();
            
            // 检查并应用降落规则（到达目标后降落）
            checkAndApplyLandingRules();
        }
    }
    
    /**
     * 持续快速更新模型（后台线程）
     */
    private void continuousModelUpdate() {
        try {
            int currentSiteCount = worldPart.getSites().size();
            int updateCount = 0;
            
            while (true) {
                updateCount++;
                
                // 获取最新的 World Model
            PureBigraph latestWorld = fetchWorldModelFromService();
            int newSiteCount = latestWorld.getSites().size();
                
                // 根据 ROS2 位置或站点数量变化，重新创建 Drone Model
                boolean needUpdateDrone = (newSiteCount != currentSiteCount) || 
                                         (rosUpdateEnabled && !dronePositions.isEmpty() && !collisionRiskDetected.get());
                
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
                
                // 标记首次更新完成
                if (!firstUpdateCompleted.get()) {
                    firstUpdateCompleted.set(true);
                }
                
                // 更新固定延迟，毫秒。
                TimeUnit.MILLISECONDS.sleep(1000);
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
                .add("Ref", 1);
        for (int i = 0; i <= 10; i++) {
            builder.add("D" + i, 0);
        }
        return builder.create();
    }

    /**
     * 从本地获取World Model签名（不转换为Bigraph）
     */
    private DynamicSignature fetchWorldSignatureFromService() throws Exception {
        if (serviceWorldSignature != null) {
            return serviceWorldSignature;
        }
        
        System.out.println("\n==================Local World Signature======================");
        
        // 从本地签名文件加载
            List<org.eclipse.emf.ecore.EObject> sigObjects = BigraphFileModelManagement.Load.signatureInstanceModel(
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
        
        System.out.println("\nFetching BiGrid instance model...");
        URI uri = URI.create(String.format(Locale.ROOT,
                "%s/generate/diagonal-directional/bigrid?rows=%d&cols=%d&format=%s",
                bigridServiceBaseUrl, bigridRows, bigridCols, bigridFormat));
        
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
            List<org.eclipse.emf.ecore.EObject> worldObjects = BigraphFileModelManagement.Load.bigraphInstanceModel(metaModel, inputStream);
            
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
                "{\"x\":%.3f,\"y\":%.3f,\"stepSizeX\":%.3f,\"stepSizeY\":%.3f}",
                gridOriginX, gridOriginY, gridStepX, gridStepY);
    }

    private PureBigraph droneModel(int siteCount) throws InvalidConnectionException, TypeNotExistsException {
        if (siteCount <= 0) {
            return pureBuilder(sig()).create();
        }

        List<Bigraph<DynamicSignature>> placements = new ArrayList<>();
        for (int i = 0; i < siteCount; i++) {
            placements.add(emptyOccupiedCell()); 
        }

        int dronesToPlace = Math.min(configuredDroneCount, siteCount);
        for (int idx = 0; idx < dronesToPlace; idx++) {
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
        
        // 获取无人机的实际电池和通信状态
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
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("Landed");

        reactumB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Status").down()
                .child("flying");
        InstantiationMap instantiationMap = InstantiationMap.create(2);
        instantiationMap.map(0, 0);
        instantiationMap.map(1, 1);
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

    private ParametricReactionRule<PureBigraph> droneBatteryLevelRule(String id) throws Exception {
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
                .child("Battery").down()
                .child("Normal");

        reactumB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Battery").down()
                .child("Low");
        InstantiationMap instantiationMap = InstantiationMap.create(2);
        instantiationMap.map(0, 0);
        instantiationMap.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
    }

    private ParametricReactionRule<PureBigraph> droneCommunicationNormalToBadRule(String id) throws Exception {
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
                .child("Communication").down()    
                .child("Normal");

        reactumB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Communication").down()
                .child("Bad");
        InstantiationMap instantiationMap = InstantiationMap.create(2);
        instantiationMap.map(0, 0);
        instantiationMap.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
    }

    private ParametricReactionRule<PureBigraph> droneCommunicationBadToNormalRule(String id) throws Exception {
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
                .child("Communication").down()    
                .child("Bad");

        reactumB.root()
                .child("OccupiedBy").down()
                .site()
                .child("Drone", normalizedId).down()
                .site()
                .child("ID").down()
                .child(id).up()
                .child("Communication").down()
                .child("Normal");
        InstantiationMap instantiationMap = InstantiationMap.create(2);
        instantiationMap.map(0, 0);
        instantiationMap.map(1, 1);
        return new ParametricReactionRule<>(redexB.create(), reactumB.create(), instantiationMap);
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
     * 移动规则：向前移动（X增加方向）
     */
    private ParametricReactionRule<PureBigraph> droneMoveForwardRule(String id) throws Exception {
        return createDirectionalMoveRule(id, "ForwardRoute");
    }

    /**
     * 移动规则：向后移动（X减少方向）
     */
    private ParametricReactionRule<PureBigraph> droneMoveBackRule(String id) throws Exception {
        return createDirectionalMoveRule(id, "BackRoute");
    }

    /**
     * 移动规则：向左移动（Y增加方向）
     */
    private ParametricReactionRule<PureBigraph> droneMoveLeftRule(String id) throws Exception {
        return createDirectionalMoveRule(id, "LeftRoute");
    }

    /**
     * 移动规则：向右移动（Y减少方向）
     */
    private ParametricReactionRule<PureBigraph> droneMoveRightRule(String id) throws Exception {
        return createDirectionalMoveRule(id, "RightRoute");
    }
    
    /**
     * 移动规则：左前移动（X增加，Y增加）
     */
    private ParametricReactionRule<PureBigraph> droneMoveForwardLeftRule(String id) throws Exception {
        return createDirectionalMoveRule(id, "ForwardLeftRoute");
    }
    
    /**
     * 移动规则：右前移动（X增加，Y减少）
     */
    private ParametricReactionRule<PureBigraph> droneMoveForwardRightRule(String id) throws Exception {
        return createDirectionalMoveRule(id, "ForwardRightRoute");
    }
    
    /**
     * 移动规则：左后移动（X减少，Y增加）
     */
    private ParametricReactionRule<PureBigraph> droneMoveBackLeftRule(String id) throws Exception {
        return createDirectionalMoveRule(id, "BackLeftRoute");
    }
    
    /**
     * 移动规则：右后移动（X减少，Y减少）
     */
    private ParametricReactionRule<PureBigraph> droneMoveBackRightRule(String id) throws Exception {
        return createDirectionalMoveRule(id, "BackRightRoute");
    }


    /**
     * 发送HTTP POST请求到无人机控制服务（通用方法）
     * @param droneId 无人机ID
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
            
            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
            
            if (response.statusCode() == 200) {
                System.out.println("  ✓ " + droneId + " " + successMessage);
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
            
            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
            
            if (response.statusCode() == 200) {
                System.out.println("  ✓ " + droneId + " navigation command sent -> (" + 
                        String.format("%.2f, %.2f, %.2f", x, y, z) + ")");
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
     */
    private void performTakeoffSequence(PureBigraph compositeModel) throws Exception {
        if (!droneControlEnabled) {
            System.out.println("\n========================================");
            System.out.println("Drone control function not enabled");
            System.out.println("To enable, set: drone.control.enabled=true");
            System.out.println("========================================\n");
            return;
        }
        
        System.out.println("\n========================================");
        System.out.println("Starting takeoff sequence: rule matching and takeoff control");
        System.out.println("========================================");
        
        AbstractBigraphMatcher<PureBigraph> matcher = AbstractBigraphMatcher.create(PureBigraph.class);
        
        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            int cfNumber = 231 + i;
            int port = droneControlStartPort + i;
            
            try {
                System.out.println("\nProcessing " + droneId + " (cf" + cfNumber + ", port " + port + "):");
                
                // 创建起飞规则
                ParametricReactionRule<PureBigraph> takeoffRule = droneTakeOffRule(droneId);
                
                // 尝试匹配规则
                Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(compositeModel, takeoffRule).iterator();
                
                if (matchIterator.hasNext()) {
                    System.out.println("  ✓ Rule matched successfully - " + droneId + " status is Landed, can take off");
                    
                    // 发送起飞指令
                    System.out.println("  → Sending takeoff commands...");
                    
                    // 步骤1: 激活 idle 状态
                    if (!activateIdle(droneId, port)) {
                        System.err.println("  !! Unable to activate idle state, skipping " + droneId);
                        continue;
                    }
                    
                    TimeUnit.MILLISECONDS.sleep(500);  // 等待状态稳定
                    
                    // 步骤2: 开始起飞
                    if (!beginTakeoff(droneId, port)) {
                        System.err.println("  !! Unable to send takeoff command, skipping " + droneId);
                        continue;
                    }
                    
                    System.out.println("  ✓ " + droneId + " takeoff sequence initiated");
                } else {
                    System.out.println("  ○ Rule not matched - " + droneId + " may already be in flying status");
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
     * 移动方向枚举
     * 坐标系统：Grid0在右下角，向左Y增加，向前X增加（与rviz一致）
     */
    private enum MoveDirection {
        FORWARD,          // 向前（X增加）
        BACK,             // 向后（X减少）
        LEFT,             // 向左（Y增加）
        RIGHT,            // 向右（Y减少）
        FORWARD_LEFT,     // 左前（X增加，Y增加）
        FORWARD_RIGHT,    // 右前（X增加，Y减少）
        BACK_LEFT,        // 左后（X减少，Y增加）
        BACK_RIGHT        // 右后（X减少，Y减少）
    }
    
    /**
     * 判断从当前点到下一点的移动方向
     * @param current 当前网格点
     * @param next 下一个网格点
     * @return 移动方向（包括8个方向：前后左右+4个对角线）
     */
    private MoveDirection getMoveDirection(GridPoint current, GridPoint next) {
        // 计算X和Y的变化
        double deltaX = next.x - current.x;
        double deltaY = next.y - current.y;
        
        // 判断是否为对角线移动（X和Y同时变化）
        boolean xChanged = Math.abs(deltaX) > 0.01;  // 使用容差判断
        boolean yChanged = Math.abs(deltaY) > 0.01;
        
        if (xChanged && yChanged) {
            // 对角线移动：4个方向
            if (deltaX > 0 && deltaY > 0) {
                return MoveDirection.FORWARD_LEFT;  // 左前
            } else if (deltaX > 0 && deltaY < 0) {
                return MoveDirection.FORWARD_RIGHT;  // 右前
            } else if (deltaX < 0 && deltaY > 0) {
                return MoveDirection.BACK_LEFT;  // 左后
            } else {
                return MoveDirection.BACK_RIGHT;  // 右后
            }
        } else if (xChanged) {
            // 只有X变化
            return deltaX > 0 ? MoveDirection.FORWARD : MoveDirection.BACK;
        } else if (yChanged) {
            // 只有Y变化
            return deltaY > 0 ? MoveDirection.LEFT : MoveDirection.RIGHT;
        }
        
        // 没有移动（理论上不应该发生）
        return MoveDirection.FORWARD;
    }
    
    /**
     * 根据移动方向获取对应的Bigraph移动规则
     * @param droneId 无人机ID
     * @param direction 移动方向
     * @return 对应的移动规则（包括直线和对角线移动）
     */
    private ParametricReactionRule<PureBigraph> getMoveRuleForDirection(String droneId, MoveDirection direction) throws Exception {
        switch (direction) {
            case FORWARD:
                return droneMoveForwardRule(droneId);
            case BACK:
                return droneMoveBackRule(droneId);
            case LEFT:
                return droneMoveLeftRule(droneId);
            case RIGHT:
                return droneMoveRightRule(droneId);
            case FORWARD_LEFT:
                return droneMoveForwardLeftRule(droneId);
            case FORWARD_RIGHT:
                return droneMoveForwardRightRule(droneId);
            case BACK_LEFT:
                return droneMoveBackLeftRule(droneId);
            case BACK_RIGHT:
                return droneMoveBackRightRule(droneId);
            default:
                throw new IllegalArgumentException("未知的移动方向: " + direction);
        }
    }
    
    /**
     * 执行导航控制：动态A*规划 + Grid预订机制 + 智能等待
     * 
     * 新的移动逻辑：
     * 1. 每次移动前动态规划最短路径的下一步（不再使用预规划的完整路径）
     * 2. 尝试匹配Bigraph规则
     * 3. 如果匹配成功，预订下一个grid（同步锁机制，防止冲突）
     * 4. 如果预订成功，发送移动指令
     * 5. 如果预订失败或规则不匹配，进入等待状态
     * 6. 等待超时后，认为前方障碍物不会移动，重新规划绕路
     */
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
                GridPoint target = droneTargets.get(droneId);
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
                    } else {
                        // 还在移动中，跳过本次导航控制
                        continue;
                    }
                }
                
                // 检查是否已到达最终目标
                if (currentPos.gridIndex == target.gridIndex) {
                    System.out.println("\n [Target Reached] " + droneId + " reached target Grid[" + target.gridIndex + "]");
                    status.reachedDestination = true;
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
                
                // 【核心逻辑1】动态A*规划：每次移动前重新规划下一步（获取最短路径）
                GridPoint nextPoint = planNextStep(droneId, currentGridPoint, target);
                
                if (nextPoint == null) {
                    System.err.println("  !! " + droneId + " unable to plan path to target");
                    status.isWaiting = true;
                    continue;
                }
                
                // 【核心逻辑2】先匹配Bigraph规则（检查是否真的能移动）
                MoveDirection direction = getMoveDirection(currentGridPoint, nextPoint);
                String directionName = direction.name();
                
                ParametricReactionRule<PureBigraph> moveRule = getMoveRuleForDirection(droneId, direction);
                Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(composite, moveRule).iterator();
                
                if (!matchIterator.hasNext()) {
                    // 规则不匹配（前方被其他无人机实际占用或无Route连接）
                    System.out.println("  ⚠ " + droneId + " move " + directionName + " rule not matched (ahead occupied)");
                    
                    // 进入等待状态
                    if (!status.isWaiting) {
                        status.isWaiting = true;
                        status.waitingStartTime = currentTime;
                        status.waitingForGrid = nextPoint.gridIndex;
                        System.out.println("[Waiting] " + droneId + " waiting for Grid[" + nextPoint.gridIndex + "] to clear");
                    }
                    
                    // 检查等待超时
                    long waitingTime = currentTime - status.waitingStartTime;
                    if (waitingTime > WAIT_TIMEOUT) {
                        System.out.println("[Wait Timeout] " + droneId + " waited over " + (WAIT_TIMEOUT/1000) + 
                                          " seconds, ahead drone not moving, preparing detour...");
                        
                        // 超时后，将被占用的grid加入障碍物，重新规划绕路
                        Set<Integer> occupiedGrids = getOccupiedGrids(droneId);
                        occupiedGrids.add(nextPoint.gridIndex);  // 将等待的grid也视为障碍物
                        
                        // 尝试规划绕路
                        List<GridPoint> detourPath = planPath(currentGridPoint, target, occupiedGrids);
                        if (!detourPath.isEmpty() && detourPath.size() > 1) {
                            GridPoint detourNextPoint = detourPath.get(1);
                            
                            // 检查绕路点是否和原来一样（说明无法绕路）
                            if (detourNextPoint.gridIndex == nextPoint.gridIndex) {
                                System.out.println("  !! Detour failed: no alternative path, continuing to wait...");
                                status.waitingStartTime = currentTime;  // 重置等待时间
                            } else {
                                System.out.println(" [Detour Success] " + droneId + " found detour: Grid[" + detourNextPoint.gridIndex + "]");
                                // 重置等待状态，下次循环会尝试新路径
                                status.isWaiting = false;
                                status.waitingForGrid = null;
                            }
                        } else {
                            // 无法绕路，重置等待时间，继续等待
                            System.out.println("!!Cannot find detour, continuing to wait...");
                            status.waitingStartTime = currentTime;
                        }
                    }
                    
                    continue;
                }
                
                // 规则匹配成功，说明可以移动
                System.out.println("  ✓ " + droneId + " move " + directionName + " rule matched successfully");
                
                // 【核心逻辑3】规则匹配后，再尝试预订grid（防止其他无人机同时移动到这里）
                boolean reservationSuccess = tryReserveGrid(droneId, nextPoint.gridIndex);
                
                if (!reservationSuccess) {
                    // Grid被其他无人机预订了（虽然Bigraph规则匹配了，但被抢先预订）
                    String reservedBy = getGridReservation(nextPoint.gridIndex);
                    System.out.println("  ⚠ " + droneId + " Grid[" + nextPoint.gridIndex + 
                                      "] reserved by " + reservedBy + ", waiting...");
                    
                    // 进入等待状态
                    if (!status.isWaiting) {
                        status.isWaiting = true;
                        status.waitingStartTime = currentTime;
                        status.waitingForGrid = nextPoint.gridIndex;
                    }
                    
                    // 检查是否超时
                    long waitingTime = currentTime - status.waitingStartTime;
                    if (waitingTime > WAIT_TIMEOUT) {
                        System.out.println("[Reservation Timeout] " + droneId + " waited over " + (WAIT_TIMEOUT/1000) + " seconds, attempting detour...");
                        
                        // 将被预订的grid视为障碍物，尝试规划绕路
                        Set<Integer> occupiedGrids = getOccupiedGrids(droneId);
                        occupiedGrids.add(nextPoint.gridIndex);  // 将被预订的grid视为障碍物
                        
                        List<GridPoint> detourPath = planPath(currentGridPoint, target, occupiedGrids);
                        if (!detourPath.isEmpty() && detourPath.size() > 1) {
                            GridPoint detourNextPoint = detourPath.get(1);
                            
                            // 检查绕路点是否和原来一样
                            if (detourNextPoint.gridIndex == nextPoint.gridIndex) {
                                System.out.println(" !! Detour failed: no alternative path, continuing to wait...");
                                status.waitingStartTime = currentTime;  // 重置等待时间
                            } else {
                                System.out.println("  🔄 [Detour Success] " + droneId + " found detour: Grid[" + detourNextPoint.gridIndex + "]");
                                status.isWaiting = false;
                                status.waitingForGrid = null;
                                // 下次循环会尝试新路径
                            }
                        } else {
                            System.out.println(" !! Cannot find detour, continuing to wait...");
                            status.waitingStartTime = currentTime;  // 重置等待时间
                        }
                    }
                    
                    continue;
                }
                
                // 预订成功！
                if (status.isWaiting) {
                    System.out.println("  ✓ [Wait End] " + droneId + " successfully reserved Grid[" + nextPoint.gridIndex + "]");
                    status.isWaiting = false;
                    status.waitingForGrid = null;
                }
                status.reservedGrid = nextPoint.gridIndex;
                
                // 【核心逻辑4】预订成功后，发送导航指令
                System.out.println(droneId + " locked Grid[" + nextPoint.gridIndex + "], preparing to move");
                
                int droneIndex = Integer.parseInt(droneId.substring(1));
                int port = droneControlStartPort + droneIndex;
                
                System.out.println("[Navigation] " + droneId + " from Grid[" + currentPos.gridIndex + 
                        "](" + String.format("%.0f,%.0f", currentGridPoint.x, currentGridPoint.y) + ") " +
                        directionName + " moving to Grid[" + nextPoint.gridIndex + 
                        "](" + String.format("%.0f,%.0f", nextPoint.x, nextPoint.y) + ")");
                
                if (navigateTo(droneId, port, nextPoint.x, nextPoint.y, 0.6)) {
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
                
            } catch (Exception e) {
                System.err.println(" !! Error processing " + droneId + " navigation: " + e.getMessage());
                e.printStackTrace();
            }
        }
    }
    
    /**
     * 应用Bigraph规则并持久化（通用方法）
     * @param droneId 无人机ID
     * @param rule 要应用的规则
     * @param logPrefix 日志前缀
     * @return 是否成功应用规则
     */
    private boolean applyRuleAndPersist(String droneId, ParametricReactionRule<PureBigraph> rule, String logPrefix) {
        try {
            PureBigraph currentComposite = composite;
            AbstractBigraphMatcher<PureBigraph> matcher = AbstractBigraphMatcher.create(PureBigraph.class);
            Iterator<BigraphMatch<PureBigraph>> matchIterator = matcher.match(currentComposite, rule).iterator();
            
            if (matchIterator.hasNext()) {
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
        } catch (Exception e) {
            e.printStackTrace();
            return false;
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
                try {
                    System.out.println("\n [Rule Apply] " + droneId + " took off (altitude: " + 
                            String.format("%.3f", status.z) + "m), applying takeoff rule...");
                    
                    ParametricReactionRule<PureBigraph> takeoffRule = droneTakeOffRule(droneId);
                    if (applyRuleAndPersist(droneId, takeoffRule, "status updated to flying")) {
                        status.status = "flying";
                        status.takeoffRuleApplied = true;
                        anyRuleApplied = true;
                    } else {
                        status.takeoffRuleApplied = true;  // 标记为已处理，避免重复检查
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
                    
                    // 获取端口
                    int droneIndex = Integer.parseInt(droneId.substring(1));
                    int port = droneControlStartPort + droneIndex;
                    
                    // 立即发送降落指令
                    if (beginLanding(droneId, port)) {
                        status.landingCommandSent = true;
                        status.landingCommandTime = currentTime;
                        System.out.println("  ✓ " + droneId + " landing command sent (time: " + currentTime + ")");
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
                        if (applyRuleAndPersist(droneId, landingRule, "status updated to Landed")) {
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
    
    /**
     * 检查并应用电池规则
     * 如果 battery_voltage < 3.2，应用 droneBatteryLevelRule
     */
    private void checkAndApplyBatteryRule(String droneId, DroneStatus status) {
        // 检查电池电压
        if (status.batteryVoltage < 3.2 && !"Low".equals(status.batteryLevel)) {
            try {
                System.out.println("\n [Battery Rule] " + droneId + " battery voltage too low: " + 
                        String.format("%.2f", status.batteryVoltage) + "V < 3.2V, applying battery rule...");
                
                ParametricReactionRule<PureBigraph> batteryRule = droneBatteryLevelRule(droneId);
                if (applyRuleAndPersist(droneId, batteryRule, "battery status updated to Low")) {
                    status.batteryLevel = "Low";
                    status.batteryRuleApplied = true;
                } else {
                    status.batteryLevel = "Low";  // 即使规则不匹配，也更新状态
                }
            } catch (Exception e) {
                System.err.println("  !! Failed to apply battery rule: " + e.getMessage());
                e.printStackTrace();
            }
        } else if (status.batteryVoltage >= 3.2 && "Low".equals(status.batteryLevel)) {
            // 电池恢复，但当前没有从Low到Normal的规则，所以只更新状态
            status.batteryLevel = "Normal";
            status.batteryRuleApplied = false;  // 重置标志，允许再次应用规则
            System.out.println("  ✓ " + droneId + " battery voltage recovered: " + 
                    String.format("%.2f", status.batteryVoltage) + "V, status updated to Normal");
        }
    }
    
    /**
     * 检查并应用通信规则
     * 如果 rssi >= 70，应用 droneCommunicationNormalToBadRule
     * 如果 rssi < 70 且当前是 Bad，应用 droneCommunicationBadToNormalRule
     */
    private void checkAndApplyCommunicationRule(String droneId, DroneStatus status) {
        try {
            // RSSI >= 70 → 通信状态变为 Bad
            if (status.rssi >= 70 && !"Bad".equals(status.communicationStatus)) {
                System.out.println("\n [Communication Rule] " + droneId + " RSSI too high: " + 
                        status.rssi + " >= 70, applying communication rule (Normal → Bad)...");
                
                ParametricReactionRule<PureBigraph> commRule = droneCommunicationNormalToBadRule(droneId);
                if (applyRuleAndPersist(droneId, commRule, "communication status updated to Bad")) {
                    status.communicationStatus = "Bad";
                    status.communicationRuleApplied = true;
                } else {
                    status.communicationStatus = "Bad";
                }
            }
            // RSSI < 70 且当前是 Bad → 通信状态恢复为 Normal
            else if (status.rssi < 70 && "Bad".equals(status.communicationStatus)) {
                System.out.println("\n [Communication Rule] " + droneId + " RSSI recovered: " + 
                        status.rssi + " < 70, applying communication rule (Bad → Normal)...");
                
                ParametricReactionRule<PureBigraph> commRule = droneCommunicationBadToNormalRule(droneId);
                if (applyRuleAndPersist(droneId, commRule, "communication status updated to Normal")) {
                    status.communicationStatus = "Normal";
                    status.communicationRuleApplied = true;
                } else {
                    status.communicationStatus = "Normal";
                }
            }
        } catch (Exception e) {
            System.err.println("  !! Failed to apply communication rule: " + e.getMessage());
            e.printStackTrace();
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
    private void subscribeRosTopic(String host, String topic, String type, RosMessageHandler handler) {
        try {
            Ros ros = new Ros(host);
            ros.connect();
            Topic rosTopic = new Topic(ros, topic, type);
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
        
        // 初始化所有无人机位置和状态为默认值
        for (int i = 0; i < configuredDroneCount; i++) {
            String droneId = "D" + i;
            // 初始化为默认位置（左下角，网格索引 i）
            dronePositions.put(droneId, new DronePosition(droneId, 0, 0, i));
            // 初始化状态
            droneStatuses.put(droneId, new DroneStatus(droneId));
        }
        System.out.println("✓ Initialized default positions and status for " + configuredDroneCount + " drones");
        
        for (int i = 0; i < configuredDroneCount; i++) {
            int droneIndex = i;
            String droneId = "D" + i;
            // cf231 对应 D0, cf232 对应 D1, 依此类推
            int cfNumber = 231 + i;
            String topic = "/cf" + cfNumber + "/pose";
            
            //创建订阅，注册回调函数，只运行一次，每次收到ROS2话题消息时，都会调用回调函数
            subscribeRosTopic(rosBridgeHost, topic, "geometry_msgs/PoseStamped", message -> {
                JsonObject jsonObject = message.toJsonObject();
                
                // 提取位置信息
                JsonObject pose = jsonObject.getJsonObject("pose");
                JsonObject position = pose.getJsonObject("position");
                double x = position.getJsonNumber("x").doubleValue();
                double y = position.getJsonNumber("y").doubleValue();
                double z = position.getJsonNumber("z").doubleValue();
                
                // 计算网格索引
                int gridIndex = coordinateToGridIndex(x, y);
                
                // 更新无人机高度状态
                DroneStatus status = droneStatuses.get(droneId);
                if (status != null) {
                    status.z = z;
                    
                    // 检查是否已经起飞（z > 0.1 且之前未标记为已起飞）
                    if (z > 0.1 && !status.hasTakenOff) {
                        status.hasTakenOff = true;
                        status.takeoffTime = System.currentTimeMillis();
                        System.out.println("[Takeoff Detection] " + droneId + " took off! Altitude: " + String.format("%.3f", z) + "m");
                    }
                    
                    // 检查是否已经降落（z <= 0.1 且之前发送过降落指令）
                    if (z <= 0.1 && status.landingCommandSent && !status.hasLanded) {
                        status.hasLanded = true;
                        System.out.println("[Landing Detection] " + droneId + " landed! Altitude: " + String.format("%.3f", z) + "m");
                    }
                }
                
                // 更新无人机位置
                DronePosition oldPos = dronePositions.get(droneId);
                DronePosition newPos = new DronePosition(droneId, x, y, gridIndex);
                
                // 只在网格索引真正变化时才更新和输出
                boolean shouldUpdate = false;
                if (oldPos == null) {
                    shouldUpdate = true;
                } else if (oldPos.gridIndex != gridIndex) {
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
            
            // 订阅状态话题（电池电压和RSSI）
            String statusTopic = "/cf" + cfNumber + "/status";
            subscribeRosTopic(rosBridgeHost, statusTopic, "crazyflie_interfaces/msg/Status", message -> {
                try {
                    JsonObject jsonObject = message.toJsonObject();
                    
                    // 提取电池电压和RSSI
                    if (jsonObject.containsKey("battery_voltage") && jsonObject.containsKey("rssi")) {
                        double batteryVoltage = jsonObject.getJsonNumber("battery_voltage").doubleValue();
                        int rssi = jsonObject.getJsonNumber("rssi").intValue();
                        
                        // 更新无人机状态
                        DroneStatus status = droneStatuses.get(droneId);
                        if (status != null) {
                            status.batteryVoltage = batteryVoltage;
                            status.rssi = rssi;
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
     * 将世界坐标转换为网格索引
     * 
     * 网格布局说明：
     * - 网格中心点坐标：gridOriginX + xIndex * gridStepX, gridOriginY + yIndex * gridStepY
     * - 网格边界：中心点 ± gridStep/2
     * 公式：index = xIndex * bigridRows + yIndex
     * 
     * @param x 世界坐标 X
     * @param y 世界坐标 Y
     * @return 网格索引（0-based），如果超出范围返回 -1
     */
    private int coordinateToGridIndex(double x, double y) {
        // 容差值，用于处理浮点数精度问题
        final double EPSILON = 1e-6;
        
        // 处理 -0.0 的情况，将其视为 0.0
        if (Math.abs(x) < EPSILON) x = 0.0;
        if (Math.abs(y) < EPSILON) y = 0.0;
        
        // 计算相对于网格原点的偏移
        double relX = x - gridOriginX;
        double relY = y - gridOriginY;
        
        // 关键修复：使用四舍五入（round）而不是向下取整（floor）
        // 因为网格坐标指的是中心点，范围是 [center - step/2, center + step/2)
        int xIndex = (int) Math.round(relX / gridStepX);
        int yIndex = (int) Math.round(relY / gridStepY);

        // 检查是否在有效范围内
        if (xIndex < 0 || xIndex >= bigridCols || yIndex < 0 || yIndex >= bigridRows) {
            return -1; // 超出网格范围
        }
        
        // 计算线性索引（X列优先，即同一X列的Y依次排列）
        int index = xIndex * bigridRows + yIndex;
        
        return index;
    }
    
    // ========================================
    // 路径规划和导航
    // ========================================
    
    /**
     * 解析目标点配置
     */
    private void parseDroneTargets() {
        System.out.println("\n========================================");
        System.out.println("Parsing drone target points");
        System.out.println("========================================");
        
        String[] targets = droneTargetsConfig.split(";");
        for (int i = 0; i < targets.length && i < configuredDroneCount; i++) {
            String[] coords = targets[i].trim().split(",");
            if (coords.length == 2) {
                try {
                    double x = Double.parseDouble(coords[0].trim());
                    double y = Double.parseDouble(coords[1].trim());
                    int gridIndex = coordinateToGridIndex(x, y);
                    String droneId = "D" + i;
                    droneTargets.put(droneId, new GridPoint(x, y, gridIndex));
                    System.out.println("  " + droneId + " target: (" + x + ", " + y + ") -> Grid[" + gridIndex + "]");
                } catch (NumberFormatException e) {
                    System.err.println("  !! Failed to parse target point: " + targets[i]);
                }
            }
        }
        System.out.println("========================================\n");
    }
    
    /**
     * 从网格索引计算中心坐标
     */
    private GridPoint gridIndexToPoint(int gridIndex) {
        if (gridIndex < 0 || gridIndex >= bigridRows * bigridCols) {
            return null;
        }
        
        int xIndex = gridIndex / bigridRows;
        int yIndex = gridIndex % bigridRows;
        
        double x = gridOriginX + xIndex * gridStepX;
        double y = gridOriginY + yIndex * gridStepY;
        
        return new GridPoint(x, y, gridIndex);
    }
    
    /**
     * 获取网格的8个邻居（上下左右+4个对角线）
     */
    private List<GridPoint> getNeighbors(int gridIndex) {
        List<GridPoint> neighbors = new ArrayList<>();
        
        int xIndex = gridIndex / bigridRows;
        int yIndex = gridIndex % bigridRows;
        
        // 8个方向：左、右、下、上、左下、左上、右下、右上
        int[][] directions = {
            {-1, 0},  // 左
            {1, 0},   // 右
            {0, -1},  // 下
            {0, 1},   // 上
            {-1, -1}, // 左下
            {-1, 1},  // 左上
            {1, -1},  // 右下
            {1, 1}    // 右上
        };
        
        for (int[] dir : directions) {
            int newX = xIndex + dir[0];
            int newY = yIndex + dir[1];
            
            if (newX >= 0 && newX < bigridCols && newY >= 0 && newY < bigridRows) {
                int newIndex = newX * bigridRows + newY;
                GridPoint point = gridIndexToPoint(newIndex);
                if (point != null) {
                    neighbors.add(point);
                }
            }
        }
        
        return neighbors;
    }
    
    /**
     * 计算两点之间的欧几里得距离（启发式函数）
     */
    private double heuristic(GridPoint a, GridPoint b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
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
        
        // 使用A*规划完整路径
        List<GridPoint> fullPath = planPath(start, goal, occupiedGrids);
        
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
                
                // 计算代价（对角线移动代价更高）
                double moveCost = (Math.abs(neighbor.x - current.point.x) > 0.5 && 
                                  Math.abs(neighbor.y - current.point.y) > 0.5) ? Math.sqrt(2) : 1.0;
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
     * 获取所有其他无人机当前占用的网格
     */
    private Set<Integer> getOccupiedGrids(String excludeDroneId) {
        Set<Integer> occupied = new HashSet<>();
        for (Map.Entry<String, DronePosition> entry : dronePositions.entrySet()) {
            if (!entry.getKey().equals(excludeDroneId)) {
                DronePosition pos = entry.getValue();
                if (pos.gridIndex >= 0) {
                    occupied.add(pos.gridIndex);
                }
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
        // 统计每个网格中的无人机数量
        Map<Integer, List<String>> gridOccupancy = new HashMap<>();
        
        for (DronePosition pos : dronePositions.values()) {
            if (pos.gridIndex >= 0) { // 只统计有效网格内的无人机
                gridOccupancy.computeIfAbsent(pos.gridIndex, k -> new ArrayList<>())
                        .add(pos.droneId);
            }
        }
        
        // 检查是否有多架无人机在同一网格
        boolean riskDetected = false;
        for (Map.Entry<Integer, List<String>> entry : gridOccupancy.entrySet()) {
            if (entry.getValue().size() > 1) {
                riskDetected = true;
                if (!collisionRiskDetected.get()) {
                    System.err.println("\n⚠⚠⚠ Collision Risk Warning ⚠⚠⚠");
                    System.err.println("Multiple drones in same grid:");
                }
                System.err.println("  Grid[" + entry.getKey() + "]: " + 
                        String.join(", ", entry.getValue()));
            }
        }
        
        if (riskDetected && !collisionRiskDetected.get()) {
            System.err.println("Temporarily not merging models to avoid conflict");
            System.err.println("⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠\n");
        }
        
        collisionRiskDetected.set(riskDetected);
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
        
        // 创建空的站点列表
        List<Bigraph<DynamicSignature>> placements = new ArrayList<>();
        for (int i = 0; i < siteCount; i++) {
            placements.add(emptyOccupiedCell());
        }
        
        // 根据 ROS2 位置放置无人机（使用实际状态）
        //遍历dronePositions Map中的所有值，pos 包含：droneId, x, y, gridIndex
        for (DronePosition pos : dronePositions.values()) {
            if (pos.gridIndex >= 0 && pos.gridIndex < siteCount) {
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
                placements.set(pos.gridIndex, 
                        buildDrone(pos.droneId, droneStatus, "OccupiedBy"));
            }
        }
        
        Bigraph<DynamicSignature> result = placements.stream()
                .reduce(pureLinkings(sig()).identity_e(), accumulator::apply);
        return (PureBigraph) result;
    }
    
    /**
     * 无人机位置信息
     */
    private static class DronePosition {
        final String droneId;
        final double x;
        final double y;
        final int gridIndex;
        
        DronePosition(String droneId, double x, double y, int gridIndex) {
            this.droneId = droneId;
            this.x = x;
            this.y = y;
            this.gridIndex = gridIndex;
        }
    }
    
    // DroneStatus 内部类，用于存储无人机状态信息
    private static class DroneStatus {
        final String droneId;
        volatile double z;                    // 当前高度
        volatile String status;                // "Landed" 或 "flying"
        volatile boolean hasTakenOff;          // 是否已经起飞
        volatile boolean takeoffRuleApplied;          // 起飞规则是否已应用
        volatile long takeoffTime;             // 起飞时间戳（毫秒）
        volatile boolean landingCommandSent;   // 降落指令是否已发送
        volatile long landingCommandTime;      // 降落指令发送时间戳（毫秒）
        volatile boolean hasLanded;            // 是否已降落
        volatile boolean landingRuleApplied;   // 降落规则是否已应用
        
        // 路径规划相关（动态A*：每次移动前规划）
        volatile boolean reachedDestination;   // 是否到达目标点
        volatile long lastMoveTime;            // 上次移动时间
        
        // 新的等待和预订机制
        volatile boolean isWaiting;            // 是否正在等待（遇到障碍或grid被预订）
        volatile long waitingStartTime;        // 开始等待的时间
        volatile Integer waitingForGrid;       // 正在等待的grid索引
        volatile Integer reservedGrid;         // 当前预订的grid索引
        
        // 移动状态跟踪
        volatile boolean isMoving;             // 是否正在移动中（已发送指令但ROS2位置还未更新）
        volatile Integer movingToGrid;         // 正在移动到的目标grid
        
        // 电池和通信状态
        volatile double batteryVoltage;        // 电池电压
        volatile int rssi;                     // 信号强度
        volatile String batteryLevel;          // "Normal" 或 "Low"
        volatile String communicationStatus;   // "Normal" 或 "Bad"
        volatile boolean batteryRuleApplied;   // 电池规则是否已应用（用于跟踪状态变化）
        volatile boolean communicationRuleApplied; // 通信规则是否已应用（用于跟踪状态变化）
        
        DroneStatus(String droneId) {
            this.droneId = droneId;
            this.z = 0.0;
            this.status = "Landed";
            this.hasTakenOff = false;
            this.takeoffRuleApplied = false;
            this.takeoffTime = 0;
            this.landingCommandSent = false;
            this.landingCommandTime = 0;
            this.hasLanded = false;
            this.landingRuleApplied = false;
            this.reachedDestination = false;
            this.lastMoveTime = 0;
            this.isWaiting = false;
            this.waitingStartTime = 0;
            this.waitingForGrid = null;
            this.reservedGrid = null;
            this.isMoving = false;
            this.movingToGrid = null;
            this.batteryVoltage = 4.2;  // 默认满电
            this.rssi = 0;
            this.batteryLevel = "Normal";
            this.communicationStatus = "Normal";
            this.batteryRuleApplied = false;
            this.communicationRuleApplied = false;
        }
    }
    
    // 网格点类
    private static class GridPoint {
        final double x;
        final double y;
        final int gridIndex;
        
        GridPoint(double x, double y, int gridIndex) {
            this.x = x;
            this.y = y;
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
            return String.format("Grid[%d](%.1f,%.1f)", gridIndex, x, y);
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
