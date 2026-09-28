package org.example;

import edu.wpi.rail.jrosbridge.messages.Message;

/**
 * Functional interface for handling ROS2 messages
 * 
 * @author Based on streaming-bigraphs example
 */
@FunctionalInterface
public interface RosMessageHandler {
    void handle(Message message) throws Exception;
}
