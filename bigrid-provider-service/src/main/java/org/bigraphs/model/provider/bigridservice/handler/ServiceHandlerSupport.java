package org.bigraphs.model.provider.bigridservice.handler;

import org.springframework.web.reactive.function.server.ServerRequest;

public abstract class ServiceHandlerSupport {

    protected int parseQueryParamAsInt(ServerRequest request, String param, int defaultValue) {
        return request.queryParam(param).map(s -> {
            try {
                return Integer.parseInt(s);
            } catch (NumberFormatException e) {
                return defaultValue;
            }
        }).orElse(defaultValue);
    }

    protected double parseQueryParamAsDouble(ServerRequest request, String param, double defaultValue) {
        return request.queryParam(param).map(s -> {
            try {
                return Double.parseDouble(s);
            } catch (NumberFormatException e) {
                return defaultValue;
            }
        }).orElse(defaultValue);
    }
}
