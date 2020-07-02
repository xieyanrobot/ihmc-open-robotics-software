package us.ihmc.atlas.behaviors;

import org.junit.jupiter.api.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionsMappingModule;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.perception.RealsensePelvisSimulator;
import us.ihmc.humanoidBehaviors.tools.perception.VisiblePlanarRegionService;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.humanoidBehaviors.ui.simulation.RobotAndMapViewer;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.nio.file.Paths;
import java.time.Duration;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.*;
import static org.junit.jupiter.api.Assertions.*;

public class AtlasLookAndStepBehaviorTest
{
   private static final boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize"));

   Supplier<PlanarRegionsList> environment = BehaviorPlanarRegionEnvironments::createRoughUpAndDownStairsWithFlatTop;

   @Test
   public void testLookAndStepOverStairs()
   {
      assertTimeoutPreemptively(Duration.ofMinutes(4), this::runTheTest);
   }

   private void runTheTest()
   {
      ThreadTools.startAsDaemon(this::reaModule, "REAModule");
      ThreadTools.startAsDaemon(this::kinematicSimulation, "KinematicsSimulation");

      BehaviorModule behaviorModule = BehaviorModule.createIntraprocess(BehaviorRegistry.of(LookAndStepBehavior.DEFINITION), createRobotModel());

      Ros2Node ros2Node = ROS2Tools.createRos2Node(INTRAPROCESS, "Helper");
      AtlasRobotModel robotModel = createRobotModel();

      RemoteHumanoidRobotInterface robot = new RemoteHumanoidRobotInterface(ros2Node, robotModel);

      Messager behaviorMessager = behaviorModule.getMessager();
      behaviorMessager.submitMessage(LookAndStepBehaviorAPI.OperatorReviewEnabled, false);

      AtomicReference<String> currentState = behaviorMessager.createInput(LookAndStepBehaviorAPI.CurrentState);

      behaviorMessager.submitMessage(BehaviorModule.API.BehaviorSelection, LookAndStepBehavior.DEFINITION.getName());

      Notification atTheTop = new Notification();
      Notification reachedOtherSide = new Notification();
      PausablePeriodicThread monitorThread = new PausablePeriodicThread("RobotStatusThread",
                                                                        0.5,
                                                                        () -> monitorThread(currentState, robot, atTheTop, reachedOtherSide));
      monitorThread.start();

      if (VISUALIZE)
      {
         new RobotAndMapViewer(createRobotModel(), ros2Node);
      }

      atTheTop.blockingPoll();
      LogTools.info("REACHED THE TOP");
      reachedOtherSide.blockingPoll();
      LogTools.info("REACHED OTHER SIDE");
   }

   private void monitorThread(AtomicReference<String> currentState,
                              RemoteHumanoidRobotInterface robot,
                              Notification atTheTop,
                              Notification reachedOtherSide)
   {
      FramePose3DReadOnly pelvisPose = robot.quickPollPoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame);
      LogTools.info("{} pose: {}", currentState.get(), pelvisPose);

      if (pelvisPose.getPosition().getX() > 2.5 && pelvisPose.getPosition().getZ() > 1.3)
      {
         atTheTop.set();
      }
      if (pelvisPose.getPosition().getX() > 6.0)
      {
         reachedOtherSide.set();
      }
   }

   private void reaModule()
   {
      LogTools.info("Creating simulated realsense stereo regions module");
      Ros2Node ros2Node = ROS2Tools.createRos2Node(INTRAPROCESS, ROS2Tools.REA_NODE_NAME);
      RealsensePelvisSimulator realsense = new RealsensePelvisSimulator(environment.get(), createRobotModel(), ros2Node);
      VisiblePlanarRegionService visiblePlanarRegionService = new VisiblePlanarRegionService(ros2Node, realsense);
      visiblePlanarRegionService.start();

      new PlanarRegionsMappingModule(INTRAPROCESS); // Start the SLAM mapper which look and step uses
   }

   private void kinematicSimulation()
   {
      LogTools.info("Creating kinematics  simulation");
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setPubSubImplementation(INTRAPROCESS);
      kinematicsSimulationParameters.setLogToFile(true);
      if (ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         kinematicsSimulationParameters.setLogToFile(false);
         kinematicsSimulationParameters.setIncomingLogsDirectory(Paths.get("/opt/BambooVideos"));
      }
      kinematicsSimulationParameters.setCreateYoVariableServer(false);
      AtlasKinematicSimulation.create(createRobotModel(), kinematicsSimulationParameters);
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
   }
}
