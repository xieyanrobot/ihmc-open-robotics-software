package us.ihmc.robotEnvironmentAwareness.communication;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;

public class SLAMModuleAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final CategoryTheme SLAM = apiFactory.createCategoryTheme("SLAM");
   private static final Category Root = apiFactory.createRootCategory(SLAM);

   private static final CategoryTheme Module = apiFactory.createCategoryTheme("Module");
   private static final CategoryTheme UI = apiFactory.createCategoryTheme("UI");
   private static final CategoryTheme StereoVision = apiFactory.createCategoryTheme("StereoVision");
   private static final CategoryTheme DepthCloud = apiFactory.createCategoryTheme("DepthCloud");
   private static final CategoryTheme SensorFrame = apiFactory.createCategoryTheme("SensorFrame");
   private static final CategoryTheme VelocityLimit = apiFactory.createCategoryTheme("VelocityLimit");
   private static final CategoryTheme PlanarRegions = apiFactory.createCategoryTheme("PlanarRegions");
   private static final CategoryTheme OcTree = apiFactory.createCategoryTheme("OcTree");
   private static final CategoryTheme Normal = apiFactory.createCategoryTheme("Normal");
   private static final CategoryTheme Buffer = apiFactory.createCategoryTheme("Buffer");
   private static final CategoryTheme Custom = apiFactory.createCategoryTheme("Custom");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");
   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   private static final TopicTheme Display = apiFactory.createTopicTheme("Display");
   
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Request = apiFactory.createTypedTopicTheme("Request");
   private static final TypedTopicTheme<Boolean> Clear = apiFactory.createTypedTopicTheme("Clear");
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Moving = apiFactory.createTypedTopicTheme("Moving");
   private static final TypedTopicTheme<Integer> Size = apiFactory.createTypedTopicTheme("Size");
   private static final TypedTopicTheme<String> Status = apiFactory.createTypedTopicTheme("Status");
   private static final TypedTopicTheme<Double> Value = apiFactory.createTypedTopicTheme("Value");
   
   public static final Topic<Boolean> RequestEntireModuleState = Root.child(Module).topic(Request);
   public static final Topic<Boolean> RequestPlanarRegions = Root.child(Module).child(PlanarRegions).topic(Request);
   
   public static final Topic<Boolean> SLAMEnable = Root.child(Module).topic(Enable);
   public static final Topic<Boolean> SLAMClear = Root.child(Module).topic(Clear);
   
   public static final Topic<Boolean> ShowPlanarRegionsMap = Root.child(UI).child(PlanarRegions).topic(Show);
   public static final Topic<Boolean> ShowSLAMOctreeMap = Root.child(UI).child(OcTree).topic(Show);
   public static final Topic<Boolean> ShowSLAMOctreeNormalMap = Root.child(UI).child(OcTree).child(Normal).topic(Show);
   public static final Topic<Boolean> ShowLatestFrame = Root.child(UI).child(DepthCloud).topic(Show);
   public static final Topic<Boolean> ShowSLAMSensorTrajectory = Root.child(UI).child(SensorFrame).topic(Show);
   public static final Topic<Boolean> SLAMVizClear = Root.child(UI).topic(Clear);
   public static final Topic<Boolean> SensorPoseHistoryClear = Root.child(UI).child(SensorFrame).topic(Clear);
   
   public static final Topic<Boolean> SensorStatus = Root.child(Module).child(SensorFrame).topic(Moving);
   public static final Topic<Boolean> VelocityLimitStatus = Root.child(Module).child(VelocityLimit).topic(Moving);
   
   public static final Topic<PlanarRegionsListMessage> SLAMPlanarRegionsState = Root.child(Module).child(PlanarRegions).topic(Data);
   public static final Topic<RandomICPSLAMParameters> SLAMParameters = Root.child(Module).topic(Parameters);
   public static final Topic<PolygonizerParameters> PolygonizerParameters = topic("PolygonizerParameters");
   public static final Topic<ConcaveHullFactoryParameters> ConcaveHullFactoryParameters = topic("ConcaveHullFactoryParameters");
   public static final Topic<PlanarRegionSegmentationParameters> PlanarRegionSegmentationParameters = topic("PlanarRegionSegmentationParameters");

   public static final Topic<DisplayType> SLAMOcTreeDisplayType = Root.child(UI).child(OcTree).topic(Display);
   
   public static final Topic<String> SLAMStatus = Root.child(Module).topic(Status);
   public static final Topic<String> QueuedBuffers = Root.child(Module).child(Buffer).topic(Status);
   
   public static final Topic<StereoVisionPointCloudMessage> StereoVisionPointCloudState = Root.child(UI).child(StereoVision).topic(Data);
   public static final Topic<StereoVisionPointCloudMessage> DepthPointCloudState = Root.child(UI).child(DepthCloud).topic(Data);
   public static final Topic<StereoVisionPointCloudMessage> IhmcSLAMFrameState = Root.child(UI).child(Buffer).topic(Data);
   public static final Topic<NormalOcTreeMessage> SLAMOctreeMapState = Root.child(UI).child(OcTree).topic(Data);
   public static final Topic<Integer> UISensorPoseHistoryFrames = Root.child(UI).child(SensorFrame).topic(Size);
   public static final Topic<StampedPosePacket> CustomizedFrameState = Root.child(UI).child(Custom).topic(Data);
   public static final Topic<Double> LatestFrameConfidenceFactor = Root.child(UI).child(SensorFrame).topic(Value);

   private static <T> Topic<T> topic(String name)
   {
      return Root.child(SLAM).topic(apiFactory.createTypedTopicTheme(name));
   }
   
   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
