package org.example;

import edu.wpi.rail.jrosbridge.messages.Message;

/**
 * 函数式接口，用于处理 ROS2 消息
 * 
 * @author Based on streaming-bigraphs example
 */
@FunctionalInterface
public interface RosMessageHandler {
    void handle(Message message) throws Exception;
}
