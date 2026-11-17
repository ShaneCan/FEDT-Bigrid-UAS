package org.example.mission;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.*;
import java.net.URI;
import java.net.http.*;
import java.time.Duration;
import java.util.*;

public final class GridService {
  public static GridGraph load(MissionConfig config, HttpClient client, ObjectMapper json)
      throws Exception {
    String url =
        config.gridUrl().replaceAll("/$", "")
            + "/generate/3d-diagonal-directional/bigrid?rows="
            + config.rows()
            + "&cols="
            + config.cols()
            + "&layers="
            + config.layers()
            + "&format=xml";
    String payload =
        json.writeValueAsString(
            Map.of(
                "x",
                config.origin().x(),
                "y",
                config.origin().y(),
                "z",
                config.origin().z(),
                "stepSizeX",
                config.step().x(),
                "stepSizeY",
                config.step().y(),
                "layerHeight",
                config.step().z()));
    var request =
        HttpRequest.newBuilder(URI.create(url))
            .timeout(Duration.ofSeconds(15))
            .header("Content-Type", "application/json")
            .POST(HttpRequest.BodyPublishers.ofString(payload))
            .build();
    var response = client.send(request, HttpResponse.BodyHandlers.ofString());
    if (response.statusCode() != 200)
      throw new IOException("Grid service HTTP " + response.statusCode());
    String xml = response.body().trim();
    if (xml.startsWith("{")) xml = json.readTree(xml).path("content").asText();
    return numericTopology(config, xml);
  }

  /** Read the provider's actual directed connections without constructing a Bigraph. */
  public static GridGraph numericTopology(MissionConfig config, String xml) throws Exception {
    var factory = javax.xml.parsers.DocumentBuilderFactory.newInstance();
    factory.setNamespaceAware(true);
    factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
    factory.setFeature("http://xml.org/sax/features/external-general-entities", false);
    factory.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
    factory.setAttribute(javax.xml.XMLConstants.ACCESS_EXTERNAL_DTD, "");
    factory.setAttribute(javax.xml.XMLConstants.ACCESS_EXTERNAL_SCHEMA, "");
    var root =
        factory
            .newDocumentBuilder()
            .parse(new org.xml.sax.InputSource(new StringReader(xml)))
            .getDocumentElement();
    Map<org.w3c.dom.Element, Integer> cells = new LinkedHashMap<>();
    Map<String, Integer> links = new HashMap<>();
    var all = root.getElementsByTagName("*");
    for (int i = 0; i < all.getLength(); i++) {
      var node = (org.w3c.dom.Element) all.item(i);
      if (!control(node).equals("Locale")) continue;
      var sites =
          children(node, "bChild").stream().filter(e -> control(e).equals("BSite")).toList();
      if (sites.size() != 1)
        throw new IllegalArgumentException("Locale must have exactly one indexed site");
      int cell =
          Integer.parseInt(
              sites.getFirst().getAttribute("index").isBlank()
                  ? "0"
                  : sites.getFirst().getAttribute("index"));
      if (cells.containsValue(cell)) throw new IllegalArgumentException("Duplicate cell site");
      cells.put(node, cell);
      var ports = children(node, "bPorts");
      if (ports.size() != 1
          || ports.getFirst().getAttribute("bLink").isBlank()
          || links.put(ports.getFirst().getAttribute("bLink"), cell) != null)
        throw new IllegalArgumentException("Invalid Locale identity port");
    }
    if (cells.size() != config.rows() * config.cols() * config.layers())
      throw new IllegalArgumentException("World cell count disagrees with configured geometry");
    List<GridGraph.Edge> edges = new ArrayList<>();
    for (var entry : cells.entrySet())
      for (var node : children(entry.getKey(), "bChild")) {
        String route = control(node);
        if (!route.endsWith("Route")) continue;
        var ports = children(node, "bPorts");
        Integer to = ports.size() == 1 ? links.get(ports.getFirst().getAttribute("bLink")) : null;
        if (to == null || route.equals("Route"))
          throw new IllegalArgumentException("Unresolved or nondirectional world route");
        edges.add(new GridGraph.Edge(entry.getValue(), to, route));
      }
    return new GridGraph(
        config.rows(),
        config.cols(),
        config.layers(),
        config.origin(),
        config.step(),
        config.obstacles(),
        edges);
  }

  private static String control(org.w3c.dom.Element node) {
    String type = node.getAttributeNS("http://www.w3.org/2001/XMLSchema-instance", "type");
    return type.substring(type.indexOf(':') + 1);
  }

  private static List<org.w3c.dom.Element> children(org.w3c.dom.Element node, String name) {
    List<org.w3c.dom.Element> result = new ArrayList<>();
    for (var child = node.getFirstChild(); child != null; child = child.getNextSibling())
      if (child instanceof org.w3c.dom.Element e && e.getTagName().equals(name)) result.add(e);
    return result;
  }
}
