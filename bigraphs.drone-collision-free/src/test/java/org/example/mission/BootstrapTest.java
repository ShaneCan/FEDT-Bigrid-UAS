package org.example.mission;

import static org.junit.jupiter.api.Assertions.*;

import org.example.Application;
import org.junit.jupiter.api.Test;
import org.springframework.boot.CommandLineRunner;
import org.springframework.context.annotation.*;

class BootstrapTest {
  @Test
  void oldComponentScanCannotDiscoverTheNewRunner() {
    var scan = new ClassPathScanningCandidateComponentProvider(true);
    assertTrue(
        scan.findCandidateComponents("org.example").stream()
            .noneMatch(b -> Application.class.getName().equals(b.getBeanClassName())));
    try (var context = new AnnotationConfigApplicationContext(Application.class)) {
      assertEquals(1, context.getBeansOfType(CommandLineRunner.class).size());
    }
  }
}
