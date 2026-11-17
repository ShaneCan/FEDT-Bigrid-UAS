package org.example;

import org.example.mission.*;
import org.springframework.boot.*;
import org.springframework.context.annotation.Bean;
import org.springframework.core.env.Environment;
import org.springframework.core.env.MapPropertySource;
import java.util.Map;

public class Application {
  public static void main(String[] args) {
    launch(args, true);
  }

  public static void launch(String[] args, boolean matching) {
    System.out.println("ENTRY=" + (matching ? "org.example.Application" : "org.example.ApplicationBaseline")
        + " MATCHING=" + matching + " LOCAL_AUTHORIZATION=" + matching);
    SpringApplication app = new SpringApplication(Application.class);
    app.setWebApplicationType(WebApplicationType.NONE);
    app.addInitializers(context -> context.getEnvironment().getPropertySources().addFirst(
        new MapPropertySource("entryPolicy", Map.of("mission.matching", matching,
            "mission.mode", matching ? "bigraph" : "baseline"))));
    app.run(args);
  }

  @Bean
  CommandLineRunner cooperativeMission(Environment environment) {
    return args -> {
      try (var mission = new MissionRunner(MissionConfig.from(environment), RunOptions.from(environment))) {
        mission.run();
      }
    };
  }
}
