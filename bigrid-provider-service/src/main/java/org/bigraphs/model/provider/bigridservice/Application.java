package org.bigraphs.model.provider.bigridservice;

import org.bigraphs.model.provider.bigridservice.client.BigridServiceWebClient;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.ConfigurableApplicationContext;

/**
 * Main entry point of the web app.
 *
 * @author Dominik Grzelak
 */
@SpringBootApplication
public class Application {

    public static void main(String[] args) {
        ConfigurableApplicationContext context = SpringApplication.run(Application.class, args);
        BigridServiceWebClient testClient = context.getBean(BigridServiceWebClient.class);
        System.out.println(">>> Check response (random points) = " + testClient.fetchRandomPoints().block());
    }
}
