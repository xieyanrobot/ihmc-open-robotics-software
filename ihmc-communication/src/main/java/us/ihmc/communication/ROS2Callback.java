package us.ihmc.communication;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2TopicName;
import us.ihmc.ros2.Ros2NodeInterface;
import us.ihmc.ros2.Ros2Subscription;

import java.util.function.Consumer;

/**
 * Callback listener to non-null reception of a message on a ROS 2 topic.
 *
 * @param <T> messageType
 */
public class ROS2Callback<T>
{
   private final Consumer<T> messageCallback;
   private Ros2Subscription<T> subscription;
   private volatile boolean enabled = true;

   /**
    *  For topics that use the default /ihmc/topic_name.
    */
   @Deprecated
   public ROS2Callback(Ros2NodeInterface ros2Node, Class<T> messageType, Consumer<T> messageCallback)
   {
      this(ros2Node, messageType, null, null, null, messageCallback);
   }

   public ROS2Callback(Ros2NodeInterface ros2Node,
                       Class<T> messageType,
                       String robotName,
                       String moduleTopicQualifier,
                       ROS2TopicQualifier ioTopicQualifier,
                       Consumer<T> messageCallback)
   {
      this(ros2Node, messageType, ROS2Tools.generateDefaultTopicName(messageType, robotName, moduleTopicQualifier, ioTopicQualifier), messageCallback);
   }

   public ROS2Callback(Ros2NodeInterface ros2Node, Class<T> messageType, ROS2TopicName topicName, Consumer<T> messageCallback)
   {
      this(ros2Node, messageType, topicName.type(messageType).toString(), messageCallback);
   }

   public ROS2Callback(Ros2NodeInterface ros2Node, Class<T> messageType, String topicName, Consumer<T> messageCallback)
   {
      this.messageCallback = messageCallback;
      ExceptionTools.handle(() ->
      {
         subscription = ros2Node.createSubscription(ROS2Tools.newMessageTopicDataTypeInstance(messageType), this::nullOmissionCallback, topicName);
      }, DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   private void nullOmissionCallback(Subscriber<T> subscriber)
   {
      if (enabled)
      {
         T incomingData = subscriber.takeNextData();
         if (incomingData != null)
         {
            messageCallback.accept(incomingData);
         }
         else
         {
            LogTools.warn("Received null from takeNextData()");
         }
      }
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public void destroy()
   {
      if (subscription != null)
      {
         subscription.remove();
      }
   }
}
