package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.humanoidBehaviors.tools.perception.CompositePlanarRegionService;
import us.ihmc.humanoidBehaviors.tools.perception.MultisenseHeadStereoSimulator;
import us.ihmc.humanoidBehaviors.tools.perception.RealsensePelvisSimulator;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.ArrayList;
import java.util.function.Supplier;

public class AtlasLookAndStepBehaviorDemo
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;

   private static boolean LOG_TO_FILE = Boolean.parseBoolean(System.getProperty("log.to.file"));
   private static boolean CREATE_YOVARIABLE_SERVER = Boolean.parseBoolean(System.getProperty("create.yovariable.server"));

   private final PubSubImplementation pubSubMode = PubSubImplementation.INTRAPROCESS;
   private final Supplier<PlanarRegionsList> environment = BehaviorPlanarRegionEnvironments::createRoughUpAndDownStairsWithFlatTop;
   private final Runnable simulation = this::kinematicSimulation;

   public AtlasLookAndStepBehaviorDemo()
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      ThreadTools.startAsDaemon(this::reaModule, "REAModule");
      ThreadTools.startAsDaemon(simulation, "KinematicsSimulation");

      BehaviorUIRegistry behaviorRegistry = BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION);

      BehaviorModule behaviorModule = BehaviorModule.createIntraprocess(behaviorRegistry, createRobotModel());

      LogTools.info("Creating behavior user interface");
      BehaviorUI.createIntraprocess(behaviorRegistry, createRobotModel(), behaviorModule.getMessager());
   }

   private void reaModule()
   {
      LogTools.info("Creating simulated multisense stereo regions module");
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubMode, ROS2Tools.REA_NODE_NAME);
      MultisenseHeadStereoSimulator multisense = new MultisenseHeadStereoSimulator(environment.get(), createRobotModel(), ros2Node);
      RealsensePelvisSimulator realsense = new RealsensePelvisSimulator(environment.get(), createRobotModel(), ros2Node);

      PlanarRegionSLAMMapper realsenseSLAM = new PlanarRegionSLAMMapper();

      // might be a weird delay with threads at 0.5 hz depending on each other

      ArrayList<String> topicNames = new ArrayList<>();
      topicNames.add(ROS2Tools.REALSENSE_SLAM_REGIONS.getName());
      topicNames.add(ROS2Tools.LIDAR_REA_REGIONS.getName());
      double period = 1.0;
      CompositePlanarRegionService allRegionsPublisher = new CompositePlanarRegionService(ros2Node,
                                                                                          topicNames,
                                                                                          period,
                                                                                          () -> realsenseSLAM.update(realsense.get()),
                                                                                          multisense);
      allRegionsPublisher.start();
   }

   private void dynamicsSimulation()
   {
      LogTools.info("Creating dynamics simulation");
      int recordFrequencySpeedup = 10; // Increase to 10 when you want the sims to run a little faster and don't need all of the YoVariable data.
      AtlasBehaviorSimulation.create(createRobotModel(), createCommonAvatarEnvironment(), pubSubMode, recordFrequencySpeedup).simulate();
   }

   private CommonAvatarEnvironmentInterface createCommonAvatarEnvironment()
   {
      String environmentName = PlanarRegionsListDefinedEnvironment.class.getSimpleName();
      YoAppearanceTexture cinderBlockTexture = new YoAppearanceTexture("sampleMeshes/cinderblock.png");
      return new PlanarRegionsListDefinedEnvironment(environmentName, environment.get(), cinderBlockTexture, 0.02, false);
   }

   private void kinematicSimulation()
   {
      LogTools.info("Creating kinematics  simulation");
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setPubSubImplementation(pubSubMode);
      kinematicsSimulationParameters.setLogToFile(LOG_TO_FILE);
      kinematicsSimulationParameters.setCreateYoVariableServer(CREATE_YOVARIABLE_SERVER);
      AtlasKinematicSimulation.create(createRobotModel(), kinematicsSimulationParameters);
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints);
   }

   public static void main(String[] args)
   {
      new AtlasLookAndStepBehaviorDemo();
   }
}
