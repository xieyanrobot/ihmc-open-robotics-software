package us.ihmc.robotEnvironmentAwareness.slam;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

public class SLAMModule
{
   protected final Messager reaMessager;

   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   private static final Color LATEST_ORIGINAL_POINT_CLOUD_COLOR = Color.BEIGE;
   private static final Color SOURCE_POINT_CLOUD_COLOR = Color.BLACK;
   private static final Color LATEST_POINT_CLOUD_COLOR = Color.LIME;

   private final AtomicReference<Boolean> enable;

   private final Topic<PlanarRegionsListMessage> planarRegionsStateTopicToSubmit;
   private final AtomicReference<StereoVisionPointCloudMessage> newPointCloud = new AtomicReference<>(null);
   private final LinkedList<StereoVisionPointCloudMessage> pointCloudQueue = new LinkedList<StereoVisionPointCloudMessage>();
   private final LinkedList<Boolean> stationaryFlagQueue = new LinkedList<Boolean>();
   private final LinkedList<Boolean> reasonableVelocityFlagQueue = new LinkedList<Boolean>();

   private final RandomICPSLAM slam = new RandomICPSLAM(DEFAULT_OCTREE_RESOLUTION);

   private ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private static final int THREAD_PERIOD_MILLISECONDS = 1;
   private ScheduledFuture<?> scheduledMain;
   private ScheduledFuture<?> scheduledSLAM;

   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;

   protected final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);

   private final AtomicReference<Boolean> robotStatus;
   private final AtomicReference<Boolean> velocityStatus;
   protected final AtomicLong latestRobotTimeStamp = new AtomicLong();
   protected IHMCROS2Publisher<StampedPosePacket> estimatedPelvisPublisher = null;
   protected RigidBodyTransform sensorPoseToPelvisTransformer = null;

   public SLAMModule(Messager messager, File configurationFile)
   {
      this.reaMessager = messager;

      ROS2Tools.createCallbackSubscription(ros2Node, StereoVisionPointCloudMessage.class, "/ihmc/stereo_vision_point_cloud", this::handlePointCloud);
      ROS2Tools.createCallbackSubscription(ros2Node, StereoVisionPointCloudMessage.class, "/ihmc/stereo_vision_point_cloud_D435", this::handlePointCloud);

      reaMessager.submitMessage(SLAMModuleAPI.UISensorPoseHistoryFrames, 1000);

      enable = reaMessager.createInput(SLAMModuleAPI.SLAMEnable, true);
      planarRegionsStateTopicToSubmit = SLAMModuleAPI.SLAMPlanarRegionsState;

      reaMessager.registerTopicListener(SLAMModuleAPI.SLAMParameters, slam::updateIcpSlamParameters);
      reaMessager.registerTopicListener(SLAMModuleAPI.ConcaveHullFactoryParameters, slam::setConcaveHullFactoryParameters);
      reaMessager.registerTopicListener(SLAMModuleAPI.PolygonizerParameters, slam::setPolygonizerParameters);
      reaMessager.registerTopicListener(SLAMModuleAPI.PlanarRegionSegmentationParameters, slam::setPlanarRegionSegmentationParameters);
      reaMessager.registerTopicListener(SLAMModuleAPI.NormalEstimationParameters, slam::setNormalEstimationParameters);

      reaMessager.registerTopicListener(SLAMModuleAPI.SLAMClear, (content) -> clearSLAM());

      planarRegionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REALSENSE_SLAM_MAP);

      robotStatus = reaMessager.createInput(SLAMModuleAPI.SensorStatus, false);
      velocityStatus = reaMessager.createInput(SLAMModuleAPI.VelocityLimitStatus, true);
   }

   public void start() throws IOException
   {
      if (scheduledMain == null)
      {
         scheduledMain = executorService.scheduleAtFixedRate(this::updateMain, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }

      if (scheduledSLAM == null)
      {
         scheduledSLAM = executorService.scheduleAtFixedRate(this::updateSLAM, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop() throws Exception
   {
      reaMessager.closeMessager();

      if (scheduledMain != null)
      {
         scheduledMain.cancel(true);
         scheduledMain = null;
      }

      if (scheduledSLAM != null)
      {
         scheduledSLAM.cancel(true);
         scheduledSLAM = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   private boolean isMainThreadInterrupted()
   {
      return Thread.interrupted() || scheduledMain == null || scheduledMain.isCancelled();
   }

   private boolean isSLAMThreadInterrupted()
   {
      return Thread.interrupted() || scheduledSLAM == null || scheduledSLAM.isCancelled();
   }

   public void updateSLAM()
   {
      if (isSLAMThreadInterrupted())
         return;

      if (pointCloudQueue.size() == 0)
         return;

      StereoVisionPointCloudMessage pointCloudToCompute = pointCloudQueue.getFirst();
      boolean stationaryFlag = stationaryFlagQueue.getFirst();
      boolean reasonableVelocityFlag = reasonableVelocityFlagQueue.getFirst();

      String stringToReport = "";
      boolean success;
      if (slam.isEmpty())
      {
         slam.addKeyFrame(pointCloudToCompute);
         success = true;
      }
      else
      {
         if (reasonableVelocityFlag)
         {
            if (stationaryFlag)
            {
               slam.addKeyFrame(pointCloudToCompute);
               success = true;
            }
            else
            {
               success = slam.addFrame(pointCloudToCompute);
            }
         }
         else
         {
            success = false;
         }
      }

      pointCloudQueue.removeFirst();
      stationaryFlagQueue.removeFirst();
      reasonableVelocityFlagQueue.removeFirst();
      reaMessager.submitMessage(SLAMModuleAPI.QueuedBuffers, pointCloudQueue.size() + " [" + slam.getSensorPoses().size() + "]");
      stringToReport = stringToReport + success + " " + slam.getSensorPoses().size() + " " + slam.getComputationTimeForLatestFrame() + " (sec) ";
      reaMessager.submitMessage(SLAMModuleAPI.SLAMStatus, stringToReport);

      if (success)
      {
         NormalOcTree octreeMap = slam.getOctree();
         NormalOcTreeMessage octreeMessage = OcTreeMessageConverter.convertToMessage(octreeMap);
         reaMessager.submitMessage(SLAMModuleAPI.SLAMOctreeMapState, octreeMessage);

         slam.updatePlanarRegionsMap();
         PlanarRegionsList planarRegionsMap = slam.getPlanarRegionsMap();
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsMap);
         reaMessager.submitMessage(planarRegionsStateTopicToSubmit, planarRegionsListMessage);
         planarRegionPublisher.publish(planarRegionsListMessage);

         SLAMFrame latestFrame = slam.getLatestFrame();
         Point3DReadOnly[] originalPointCloud = latestFrame.getOriginalPointCloud();
         Point3DReadOnly[] correctedPointCloud = latestFrame.getPointCloud();
         Point3DReadOnly[] sourcePointsToWorld = slam.getSourcePointsToWorldLatestFrame();
         if (originalPointCloud == null || sourcePointsToWorld == null || correctedPointCloud == null)
            return;
         StereoVisionPointCloudMessage latestStereoMessage = createLatestFrameStereoVisionPointCloudMessage(originalPointCloud,
                                                                                                            sourcePointsToWorld,
                                                                                                            correctedPointCloud);
         RigidBodyTransformReadOnly sensorPose = latestFrame.getSensorPose();
         latestStereoMessage.getSensorPosition().set(sensorPose.getTranslation());
         latestStereoMessage.getSensorOrientation().set(sensorPose.getRotation());
         reaMessager.submitMessage(SLAMModuleAPI.IhmcSLAMFrameState, latestStereoMessage);
         reaMessager.submitMessage(SLAMModuleAPI.LatestFrameConfidenceFactor, latestFrame.getConfidenceFactor());

         if (estimatedPelvisPublisher != null)
         {
            StampedPosePacket posePacket = new StampedPosePacket();
            posePacket.setTimestamp(latestRobotTimeStamp.get());
            int maximumBufferOfQueue = 10;
            if (pointCloudQueue.size() >= maximumBufferOfQueue)
            {
               posePacket.setConfidenceFactor(0.0);
            }
            else
            {
               posePacket.setConfidenceFactor(latestFrame.getConfidenceFactor());
            }
            RigidBodyTransform estimatedPelvisPose = new RigidBodyTransform(sensorPoseToPelvisTransformer);
            estimatedPelvisPose.preMultiply(sensorPose);
            posePacket.getPose().set(estimatedPelvisPose);
            reaMessager.submitMessage(SLAMModuleAPI.CustomizedFrameState, posePacket);
            estimatedPelvisPublisher.publish(posePacket);
         }
      }
   }

   private StereoVisionPointCloudMessage createLatestFrameStereoVisionPointCloudMessage(Point3DReadOnly[] originalPointCloud,
                                                                                        Point3DReadOnly[] sourcePointsToWorld,
                                                                                        Point3DReadOnly[] correctedPointCloud)
   {
      int numberOfPointsToPack = originalPointCloud.length + sourcePointsToWorld.length + correctedPointCloud.length;

      Point3D[] pointCloudBuffer = new Point3D[numberOfPointsToPack];
      int[] colorBuffer = new int[numberOfPointsToPack];
      for (int i = 0; i < originalPointCloud.length; i++)
      {
         pointCloudBuffer[i] = new Point3D(originalPointCloud[i]);
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_ORIGINAL_POINT_CLOUD_COLOR);
      }
      for (int i = originalPointCloud.length; i < originalPointCloud.length + sourcePointsToWorld.length; i++)
      {
         pointCloudBuffer[i] = new Point3D(sourcePointsToWorld[i - originalPointCloud.length]);
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(SOURCE_POINT_CLOUD_COLOR);
      }
      for (int i = originalPointCloud.length + sourcePointsToWorld.length; i < numberOfPointsToPack; i++)
      {
         pointCloudBuffer[i] = new Point3D(correctedPointCloud[i - originalPointCloud.length - sourcePointsToWorld.length]);
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_POINT_CLOUD_COLOR);
      }
      return PointCloudCompression.compressPointCloud(19870612L, pointCloudBuffer, colorBuffer, numberOfPointsToPack, 0.001, null);
   }

   public void updateMain()
   {
      if (isMainThreadInterrupted())
         return;

      if (enable.get())
      {
         StereoVisionPointCloudMessage pointCloud = newPointCloud.getAndSet(null);
         if (pointCloud == null)
            return;

         pointCloudQueue.add(pointCloud);
         stationaryFlagQueue.add(robotStatus.get());
         reasonableVelocityFlagQueue.add(velocityStatus.get());
      }
   }

   public void clearSLAM()
   {
      pointCloudQueue.clear();
      stationaryFlagQueue.clear();
      reasonableVelocityFlagQueue.clear();
      slam.clear();
      newPointCloud.set(null);
   }

   private void handlePointCloud(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      newPointCloud.set(message);
      reaMessager.submitMessage(SLAMModuleAPI.DepthPointCloudState, new StereoVisionPointCloudMessage(message));
   }

   public static SLAMModule createIntraprocessModule(String configurationFilePath) throws Exception
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

      return new SLAMModule(messager, configurationFile);
   }
}
