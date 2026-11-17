package org.example;

import edu.wpi.rail.jrosbridge.messages.Message;

@FunctionalInterface
public interface RosMessageHandler {
    void handle(Message message) throws Exception;
}
