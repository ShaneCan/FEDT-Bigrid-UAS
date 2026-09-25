package org.bigraphs.model.provider.bigridservice;

import org.bigraphs.model.provider.bigridservice.client.BigridServiceWebClient;
import org.bigraphs.model.provider.bigridservice.data.request.GridSpecRequest;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import reactor.test.StepVerifier;

import java.util.Base64;

@SpringBootTest
@Disabled
class ApplicationTests {

    @Autowired
    private BigridServiceWebClient bigridServiceWebClient;

    @Test
    void contextLoads() {
    }

    @Test
    void testFetchDefaultBigridXMI() {
        int rows = 2;
        int cols = 2;
        String format = "xml";

        StepVerifier.create(
                        bigridServiceWebClient
                                .fetchDefaultBigrid(rows, cols, format)
                                .doOnNext(response -> System.out.println("Response:\n" + response.getContent()))
                )
                .expectNextMatches(response ->
                        response.getCols() == cols &&
                                response.getRows() == rows &&
                                response.getContent() != null &&
                                !response.getContent().isEmpty())
                .verifyComplete();
    }

    @Test
    void testFetchDefaultBigridJson() {
        int rows = 2;
        int cols = 2;
        String format = "json";

        StepVerifier.create(bigridServiceWebClient.fetchDefaultBigrid(rows, cols, format)
                        .doOnNext(response -> System.out.println("Response:\n" + response.getContent())))
                .expectNextMatches(response ->
                        response.getCols() == cols &&
                                response.getRows() == rows &&
                                response.getContent() != null &&
                                !response.getContent().isEmpty())
                .verifyComplete();
    }

    @Test
    void testFetchDefaultBigridProtobuf() {
        int rows = 2;
        int cols = 2;
        String format = "protobuf";

        StepVerifier.create(bigridServiceWebClient.fetchDefaultBigrid(rows, cols, format).doOnNext(response -> System.out.println("Response:\n" + response.getContent())))
                .expectNextMatches(response ->
                        response.getCols() == cols &&
                                response.getRows() == rows &&
                                response.getContent() != null &&
                                !response.getContent().isEmpty() &&
                                isValidProtobuf(response.getContent()))
                .verifyComplete();
    }

    private boolean isValidProtobuf(String base64EncodedProto) {
        try {
            byte[] decoded = Base64.getDecoder().decode(base64EncodedProto);
            org.swarmwalker.messages.BiGrid biGrid = org.swarmwalker.messages.BiGrid.parseFrom(decoded);
            return biGrid.getItemsCount() > 0; // ToDo: other checks
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    @Test
    void testPostBigrid() {
        GridSpecRequest spec = new GridSpecRequest();
        spec.x = 0;
        spec.y = 0;
        spec.stepSizeX = 1;
        spec.stepSizeY = 1;

        int rows = 3;
        int cols = 3;
        String format = "json";

        StepVerifier.create(bigridServiceWebClient.fetchBigrid(spec, rows, cols, format))
                .expectNextMatches(response ->
                        response.getCols() == cols &&
                                response.getRows() == rows &&
                                response.getContent() != null &&
                                response.getContent().contains("v0")) // or other simple check
                .verifyComplete();
    }
}
