package org.example.mission;

import org.springframework.core.env.Environment;

/** Entry policy and scene data. No saved-plan startup dependency. */
public record RunOptions(
    boolean matching, String experimentUrl, Scenario scenario, HardwareHealth.Config health) {
  public RunOptions(boolean matching, String experimentUrl, Scenario scenario) {
    this(matching, experimentUrl, scenario, HardwareHealth.Config.defaults());
  }

  public static RunOptions normal() {
    return new RunOptions(true, "", Scenario.none());
  }

  public static RunOptions from(Environment env) {
    var options =
        new RunOptions(
            env.getProperty("mission.matching", Boolean.class, true),
            env.getProperty("experiment.service-url", ""),
            Scenario.from(env),
            HardwareHealth.Config.from(env));
    if (!options.experiment() && !options.scenario.events().isEmpty())
      throw new IllegalArgumentException("experiment.events requires experiment.service-url");
    return options;
  }

  public boolean experiment() {
    return !experimentUrl.isBlank();
  }
}
