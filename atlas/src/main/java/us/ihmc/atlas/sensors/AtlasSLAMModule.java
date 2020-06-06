package us.ihmc.atlas.sensors;

import java.io.File;
import java.io.IOException;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasSLAMModule extends SLAMModule
{
   private void handleRobotConfigurationData(Subscriber<RobotConfigurationData> subscriber)
   {
      RobotConfigurationData robotConfigurationData = subscriber.takeNextData();
      latestRobotTimeStamp.set(robotConfigurationData.getMonotonicTime());

      if (robotConfigurationData.getPelvisLinearVelocity().lengthSquared() < 0.001)
      {
         reaMessager.submitMessage(SLAMModuleAPI.SensorStatus, true);
      }
      else
      {
         reaMessager.submitMessage(SLAMModuleAPI.SensorStatus, false);
      }

      if (robotConfigurationData.getPelvisLinearVelocity().lengthSquared() < 0.01)
      {
         reaMessager.submitMessage(SLAMModuleAPI.VelocityLimitStatus, true);
      }
      else
      {
         reaMessager.submitMessage(SLAMModuleAPI.VelocityLimitStatus, false);
      }
   }

   private void handleFootstepStatusMessage(Subscriber<FootstepStatusMessage> subscriber)
   {
      FootstepStatusMessage footstepStatusMessage = subscriber.takeNextData();
      if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         RobotSide robotSide = RobotSide.fromByte(footstepStatusMessage.getRobotSide());
         Point3DReadOnly footLocation = footstepStatusMessage.getActualFootPositionInWorld();
         QuaternionReadOnly footOrientation = footstepStatusMessage.getActualFootOrientationInWorld();

         FootstepDataMessage footstepDataMessageToSubmit = new FootstepDataMessage();
         footstepDataMessageToSubmit.setRobotSide(robotSide.toByte());
         footstepDataMessageToSubmit.getLocation().set(footLocation);
         footstepDataMessageToSubmit.getOrientation().set(footOrientation);
         reaMessager.submitMessage(SLAMModuleAPI.FootstepDataState, footstepDataMessageToSubmit);
      }
   }

   public AtlasSLAMModule(Messager messager, DRCRobotModel drcRobotModel, File configurationFile)
   {
      super(messager, configurationFile);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()),
                                                    this::handleRobotConfigurationData);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    FootstepStatusMessage.class,
                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()),
                                                    this::handleFootstepStatusMessage);

      estimatedPelvisPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                    StampedPosePacket.class,
                                                                    ROS2Tools.getControllerOutputTopic(drcRobotModel.getSimpleRobotName()));
      sensorPoseToPelvisTransformer = new RigidBodyTransform(AtlasSensorInformation.transformPelvisToDepthCamera);
      sensorPoseToPelvisTransformer.invert();
   }

   public static AtlasSLAMModule createIntraprocessModule(DRCRobotModel drcRobotModel, String configurationFilePath) throws Exception
   {
      KryoMessager messager = KryoMessager.createIntraprocess(SLAMModuleAPI.API,
                                                              NetworkPorts.SLAM_MODULE_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      messager.setAllowSelfSubmit(true);
      messager.startMessager();

      File configurationFile = new File(configurationFilePath);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }

      return new AtlasSLAMModule(messager, drcRobotModel, configurationFile);
   }
}
