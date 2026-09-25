package org.example;

import org.bigraphs.model.bigraphBaseModel.impl.BigraphBaseModelPackageImpl;

/**
 * A* baseline entry: same ROS2 / REST control as {@link Application3D}, but skips per-step
 * Bigraph match and grid reservation. Run with:
 * {@code ./mvnw spring-boot:run -Dspring-boot.run.mainClass=org.example.Application3DBaseline}
 */
public class Application3DBaseline {

    public static void main(String[] args) {
        System.setProperty("baseline.mode", "true");
        BigraphBaseModelPackageImpl.init();
        org.springframework.boot.SpringApplication.run(Application3D.class, args);
    }
}
