package us.ihmc.robotEnvironmentAwareness.communication;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;

public class SLAMModuleAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("SLAM"));

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
   private static final CategoryTheme Footstep = apiFactory.createCategoryTheme("Footstep");
   private static final CategoryTheme DataManager = apiFactory.createCategoryTheme("DataManager");
   private static final CategoryTheme Export = apiFactory.createCategoryTheme("Export");
   private static final CategoryTheme Import = apiFactory.createCategoryTheme("Import");

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
   private static final TypedTopicTheme<Boolean> Save = apiFactory.createTypedTopicTheme("Save");

   private static final TypedTopicTheme<String> Path = apiFactory.createTypedTopicTheme("Path");

   public static final Topic<Boolean> SaveConfiguration = Root.child(Export).topic(Save);

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

   public static final Topic<Boolean> ShowFootstepDataViz = Root.child(UI).child(Footstep).topic(Enable);
   public static final Topic<Boolean> ClearFootstepDataViz = Root.child(UI).child(Footstep).topic(Clear);
   public static final Topic<FootstepDataMessage> FootstepDataState = Root.child(UI).child(Footstep).topic(Data);

   public static final Topic<Boolean> SensorStatus = Root.child(Module).child(SensorFrame).topic(Moving);
   public static final Topic<Boolean> VelocityLimitStatus = Root.child(Module).child(VelocityLimit).topic(Moving);

   public static final Topic<PlanarRegionsListMessage> SLAMPlanarRegionsState = Root.child(Module).child(PlanarRegions).topic(Data);
   public static final Topic<RandomICPSLAMParameters> SLAMParameters = Root.child(Module).topic(Parameters);
   
   public static final Topic<String> SLAMStatus = Root.child(Module).topic(Status);
   public static final Topic<String> QueuedBuffers = Root.child(Module).child(Buffer).topic(Status);

   public static final Topic<StereoVisionPointCloudMessage> StereoVisionPointCloudState = Root.child(UI).child(StereoVision).topic(Data);

   public static final Topic<Boolean> NormalEstimationClear = Root.child(Normal).topic(Clear);
   public static final Topic<Boolean> NormalEstimationEnable = Root.child(Normal).topic(Enable);
   public static final Topic<NormalEstimationParameters> NormalEstimationParameters = Root.child(Normal).topic(Parameters);

   public static final Topic<StereoVisionPointCloudMessage> DepthPointCloudState = Root.child(UI).child(DepthCloud).topic(Data);
   public static final Topic<StereoVisionPointCloudMessage> IhmcSLAMFrameState = Root.child(UI).child(Buffer).topic(Data);
   public static final Topic<NormalOcTreeMessage> SLAMOctreeMapState = Root.child(UI).child(OcTree).topic(Data);
   public static final Topic<Integer> UISensorPoseHistoryFrames = Root.child(UI).child(SensorFrame).topic(Size);
   public static final Topic<StampedPosePacket> CustomizedFrameState = Root.child(UI).child(Custom).topic(Data);
   public static final Topic<Double> LatestFrameConfidenceFactor = Root.child(UI).child(SensorFrame).topic(Value);

   public static final Topic<Boolean> UIRawDataExportRequest = Root.child(UI).child(DataManager).child(DepthCloud).child(Export).topic(Request);
   
   public static final Topic<String> UIRawDataExportDirectory = Root.child(UI).child(DataManager).child(DepthCloud).child(Export).topic(Path);
   public static final Topic<String> UISLAMDataExportDirectory = Root.child(UI).child(DataManager).child(Module).child(Export).topic(Path);
   
   public static final Topic<PlanarRegionsListMessage> ImportedPlanarRegionsState = Root.child(UI).child(DataManager).child(PlanarRegions).child(Import).topic(Data);
   public static final Topic<Boolean> ShowImportedPlanarRegions = Root.child(UI).child(DataManager).child(PlanarRegions).child(Import).topic(Show);
   public static final Topic<Boolean> ImportedPlanarRegionsVizClear = Root.child(UI).child(DataManager).child(PlanarRegions).child(Import).topic(Clear);
   
   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
