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
        CDOStandaloneServer server = new CDOStandaloneServer("repo1"); // 在本地开一个名为 repo1 的临时 CDO 服务器实例（无界面，供 Spring Data CDO 使用）
        CDOStandaloneServer.start(server);
    }

    @Test
    @Disabled
    void run_server_test_02() throws Exception {
        CDOStandaloneServer server = new CDOStandaloneServer(new File("src/test/resources/config/cdo-server.xml"));
        CDOStandaloneServer.start(server);
    }
}

