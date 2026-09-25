package org.bigraphs.model.provider.bigridservice.client;

import org.bigraphs.model.provider.bigridservice.data.ResponseData_GenerateGrid;
import org.bigraphs.model.provider.bigridservice.data.request.GridSpecRequest;
import org.bigraphs.model.provider.bigridservice.data.request.PointData;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.MediaType;
import org.springframework.stereotype.Service;
import org.springframework.web.reactive.function.client.WebClient;
import reactor.core.publisher.Mono;

import java.util.stream.Collectors;

/**
 * A simple service client to request the endpoints.
 *
 * @author Dominik Grzelak
 */
@Service
public class BigridServiceWebClient {

    private final WebClient client;

    // Constructor injection for dynamic address and port
    public BigridServiceWebClient(
            @Value("${server.address:localhost}") String serverAddress,
            @Value("${server.port:8080}") int serverPort
    ) {
        String baseUrl = String.format("http://%s:%d", serverAddress, serverPort);
        this.client = WebClient.create(baseUrl);
    }

    public Mono<String> fetchRandomPoints() {
        return client.get()
                .uri("/generate/points/random")
                .accept(MediaType.APPLICATION_JSON)
                .retrieve()
                .bodyToMono(PointData.class)
                .map(x -> x.getPoints().stream().map(y -> String.format("(%s, %s)",y.x,y.y)).collect(Collectors.joining(", ")));
    }

    public Mono<ResponseData_GenerateGrid> fetchDefaultBigrid(int rows, int cols, String format) {
        return client.get()
                .uri(uriBuilder -> uriBuilder
                        .path("/generate/bigrid")
                        .queryParam("rows", rows)
                        .queryParam("cols", cols)
                        .queryParam("format", format)
                        .build())
                .accept(MediaType.APPLICATION_JSON) // or other type based on `format`
                .retrieve()
                .bodyToMono(ResponseData_GenerateGrid.class);
    }

    public Mono<ResponseData_GenerateGrid> fetchBigrid(GridSpecRequest spec, int rows, int cols, String format) {
        return client.post()
                .uri(uriBuilder -> uriBuilder
                        .path("/generate/bigrid")
                        .queryParam("rows", rows)
                        .queryParam("cols", cols)
                        .queryParam("format", format)
                        .build())
                .accept(MediaType.APPLICATION_JSON) // Adjust if expecting protobuf
                .contentType(MediaType.APPLICATION_JSON)
                .bodyValue(spec)
                .retrieve()
                .bodyToMono(ResponseData_GenerateGrid.class);
    }
}