package org.example.mission;

import static org.bigraphs.framework.core.factory.BigraphFactory.*;
import static org.junit.jupiter.api.Assertions.*;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.sun.net.httpserver.HttpServer;
import java.io.*;
import java.net.*;
import java.net.http.HttpClient;
import java.util.*;
import org.bigraphs.framework.core.BigraphFileModelManagement;
import org.junit.jupiter.api.Test;
import org.springframework.mock.env.MockEnvironment;

public class GridServiceTest {
  static String worldXml() throws Exception {
    var g = GridGraph.rectangular(7, 7, 3, Set.of());
    var signature = pureSignatureBuilder().add("Locale", 1);
    for (String r : g.links().stream().map(GridGraph.Edge::route).distinct().toList())
      signature.add(r, 1);
    var b = pureBuilder(signature.create());
    var root = b.root();
    for (int c = 0; c < g.size(); c++) {
      var locale = root.child("Locale", "cell" + c).down().site();
      for (var edge : g.links())
        if (edge.from() == c) locale.child(edge.route(), "cell" + edge.to());
    }
    var out = new ByteArrayOutputStream();
    BigraphFileModelManagement.Store.exportAsInstanceModel(b.create(), out);
    return out.toString(java.nio.charset.StandardCharsets.UTF_8);
  }

  @Test
  void numericParserLoadsProviderTopologyWithoutFilesystemPaths() throws Exception {
    var server = server(0);
    try (var client = HttpClient.newHttpClient()) {
      var cfg =
          MissionConfig.from(
              new MockEnvironment()
                  .withProperty(
                      "bigrid.service.base-url",
                      "http://127.0.0.1:" + server.getAddress().getPort()));
      var g = GridService.load(cfg, client, new ObjectMapper());
      assertEquals(147, g.size());
      assertTrue(g.legal(0, 7));
      assertFalse(g.legal(17, 25));
    } finally {
      server.stop(0);
    }
  }

  @Test
  void numericParserPreservesMissingDirectedLinksAndRejectsExternalEntities() throws Exception {
    var cfg = MissionConfig.from(new MockEnvironment().withProperty("obstacle.grids", ""));
    String xml = worldXml();
    var parser = javax.xml.parsers.DocumentBuilderFactory.newInstance();
    parser.setNamespaceAware(true);
    var document =
        parser.newDocumentBuilder().parse(new org.xml.sax.InputSource(new StringReader(xml)));
    var nodes = document.getElementsByTagName("bChild");
    for (int i = 0; i < nodes.getLength(); i++) {
      var node = (org.w3c.dom.Element) nodes.item(i);
      if (node.getAttributeNS("http://www.w3.org/2001/XMLSchema-instance", "type")
          .endsWith(":ForwardRoute")) {
        node.getParentNode().removeChild(node);
        break;
      }
    }
    var writer = new StringWriter();
    javax.xml.transform.TransformerFactory.newInstance()
        .newTransformer()
        .transform(
            new javax.xml.transform.dom.DOMSource(document),
            new javax.xml.transform.stream.StreamResult(writer));
    var full = GridService.numericTopology(cfg, xml);
    var changed = GridService.numericTopology(cfg, writer.toString());
    assertEquals(full.links().size() - 1, changed.links().size());
    assertTrue(full.links().containsAll(changed.links()));
    assertThrows(
        Exception.class,
        () ->
            GridService.numericTopology(
                cfg, "<!DOCTYPE x [<!ENTITY e SYSTEM 'file:///unread'>]><x>&e;</x>"));
  }

  public static HttpServer server(int port) throws Exception {
    var mapper = new ObjectMapper();
    byte[] body =
        mapper.writeValueAsBytes(Map.of("mimeType", "application/xml", "content", worldXml()));
    var server = HttpServer.create(new InetSocketAddress("127.0.0.1", port), 0);
    server.createContext(
        "/generate/3d-diagonal-directional/bigrid",
        exchange -> {
          exchange.getRequestBody().readAllBytes();
          exchange.getResponseHeaders().set("Content-Type", "application/json");
          exchange.sendResponseHeaders(200, body.length);
          try (var out = exchange.getResponseBody()) {
            out.write(body);
          }
        });
    server.start();
    return server;
  }

  /** Isolated provider protocol fixture for simulator integration checks. */
  public static void main(String[] args) throws Exception {
    server(Integer.parseInt(args[0]));
    System.out.println("GRID FIXTURE READY");
  }
}
