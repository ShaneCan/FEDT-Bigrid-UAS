package org.example;

import org.bigraphs.spring.data.cdo.CDOStandaloneServer;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;

import java.io.File;

/**
 * @author Dominik Grzelak
 */
public class CDOStandaloneServerTest {

    @Test
    @EnabledIfSystemProperty(named = "runDisabledTests", matches = "true")
    void run_server_test_01() throws Exception {
        CDOStandaloneServer server = new CDOStandaloneServer("repo1"); // Start a local headless CDO server instance named repo1, for use by Spring Data CDO
        CDOStandaloneServer.start(server);
    }

    @Test
    @Disabled
    void run_server_test_02() throws Exception {
        CDOStandaloneServer server = new CDOStandaloneServer(new File("src/test/resources/config/cdo-server.xml"));
        CDOStandaloneServer.start(server);
    }
}

