package us.ihmc.avatar.networkProcessor.footstepPostProcessing;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingModuleLauncher;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingModule;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.BlockEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

public abstract class AvatarPostProcessingTests implements MultiRobotTestInterface
{
   private static final boolean keepSCSUp = true;

   protected SimulationTestingParameters simulationTestingParameters;
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   private FootstepPlanningModule footstepPlanningModule;
   private FootstepPlanPostProcessingModule postProcessingModule;

   private FootstepPlannerParametersBasics footstepPlannerParameters;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(keepSCSUp && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

      footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(getRobotModel(), DomainFactory.PubSubImplementation.INTRAPROCESS);
      postProcessingModule = FootstepPlanPostProcessingModuleLauncher.createModule(robotModel, PubSubImplementation.INTRAPROCESS);

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

//      if (simulationTestingParameters.getKeepSCSUp())
//      {
//         ThreadTools.sleepForever();
//      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      footstepPlanningModule.closeAndDispose();
      postProcessingModule.closeAndDispose();
      footstepPlanningModule = null;
      postProcessingModule = null;

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Test
   public void testWalkingOffOfMediumPlatform() throws SimulationExceededMaximumTimeException
   {
      double height = 0.3;
      OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup();
      startingLocation.addAdditionalOffset(new Vector3D(0.0, 0.0, height));

      BlockEnvironment blockEnvironment = new BlockEnvironment(1.0, 1.0, height);
      drcSimulationTestHelper.setTestEnvironment(blockEnvironment);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingOntoMediumPlatformToesTouchingTest");

      footstepPlannerParameters.setMaximumStepZ(height + 0.05);
      footstepPlannerParameters.setMaximumStepZWhenForwardAndDown(height - 0.05);
      footstepPlannerParameters.setMaximumStepXWhenForwardAndDown(0.22);
      footstepPlannerParameters.setIdealFootstepLength(0.28);
      footstepPlanningModule.getVisibilityGraphParameters().setTooHighToStepDistance(height + 0.05);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      Assertions.assertTrue(success);

      PoseReferenceFrame startingFrame = new PoseReferenceFrame("startingFrame", ReferenceFrame.getWorldFrame());
      startingFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), startingLocation.getAdditionalOffset()));
      startingFrame.setOrientationAndUpdate(new Quaternion(startingLocation.getYaw(), 0.0, 0.0));

      FramePose3D goalPose = new FramePose3D(startingFrame);
      goalPose.getPosition().set(1.0, 0.0, -height);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket request = getRequest(drcSimulationTestHelper.getControllerFullRobotModel(), blockEnvironment.getPlanarRegionsList(), goalPose, footstepPlannerParameters);

      runTest(request);
   }

   @Test
   public void testSwingOverPlanarRegions() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();

      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();

      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.createSimulation(className);

      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(8.0, -8.0, 5.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(1.5, 0.0, 0.8);

      footstepPlannerParameters.setMaximumStepReach(0.6);
      footstepPlannerParameters.setMinimumStepWidth(0.05);
      footstepPlannerParameters.setMaximumStepWidth(0.35);
      footstepPlannerParameters.setBodyBoxBaseZ(0.4);
      footstepPlannerParameters.setCheckForBodyBoxCollisions(false);
      footstepPlannerParameters.setCheckForPathCollisions(false);

      postProcessingModule.getParameters().setSwingOverRegionsProcessingEnabled(true);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      FramePose3D goalPose = new FramePose3D();
      goalPose.getPosition().set(2.0, 0.0, 0.0);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket requestPacket = getRequest(drcSimulationTestHelper.getControllerFullRobotModel(), environment.getPlanarRegionsList(), goalPose, footstepPlannerParameters);
      runTest(requestPacket);
   }

   @Test
   public void testWalkingOnStraightForwardLines() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();

      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      FootstepPostProcessingParametersBasics parameters = postProcessingModule.getParameters();
      parameters.setFractionLoadIfFootHasFullSupport(0.6);
      parameters.setFractionTimeOnFootIfFootHasFullSupport(0.6);
      parameters.setFractionLoadIfOtherFootHasNoWidth(0.7);
      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(0.7);

      // increase ankle damping to match the real robot better
      YoDouble damping_l_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_akx");
      YoDouble damping_l_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_aky");
      YoDouble damping_r_akx = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_akx");
      YoDouble damping_r_aky = (YoDouble) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_aky");
      damping_l_akx.set(1.0);
      damping_l_aky.set(1.0);
      damping_r_akx.set(1.0);
      damping_r_aky.set(1.0);

      SideDependentList<YoEnum<ConstraintType>> footStates = new SideDependentList<>();
      // get foot states
      for (RobotSide robotSide : RobotSide.values)
      {
         String variableName = robotSide.getCamelCaseNameForStartOfExpression() + "FootCurrentState";
         YoEnum<ConstraintType> footState = (YoEnum<ConstraintType>) drcSimulationTestHelper.getYoVariable(variableName);
         footStates.put(robotSide, footState);
      }



      // setup camera
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(-10.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      final ContactPointController contactPointController = new ContactPointController(footStates);
      drcSimulationTestHelper.addRobotControllerOnControllerThread(contactPointController);
//      setupSupportViz();

      SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
      double footForwardOffset = steppingParameters.getFootForwardOffset();
      double footBackwardOffset = steppingParameters.getFootBackwardOffset();
      double footWidth = steppingParameters.getFootWidth();
      double toeWidth = steppingParameters.getToeWidth();

      ArrayList<Point2D> soleVertices = new ArrayList<Point2D>();
      soleVertices.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      soleVertices.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      ConvexPolygon2D defaultSolePolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(soleVertices));
      defaultSolePolygon.update();


      ThreadTools.sleep(1000);


      armsUp();



      double swingDuration = 0.6;
      double transferDuration = 0.5;
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingDuration, transferDuration);

      int numberOfSteps = 2;

      for (int i = 0; i < numberOfSteps; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         ArrayList<Point2D> newContactPoints = generateContactPointsForRotatedLineOfContact(0.0, 0.0, 0.0);

         FootstepDataMessage footstepData = new FootstepDataMessage();

         ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
         FramePoint3D placeToStepInWorld = new FramePoint3D(soleFrame, 0.0, 0.0, 0.0);
         placeToStepInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         placeToStepInWorld.setX(0.3 * i);

         footstepData.getLocation().set(placeToStepInWorld);
         footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
         footstepData.setRobotSide(robotSide.toByte());
         for (Point2D contactPoint : newContactPoints)
            footstepData.getPredictedContactPoints2d().add().set(contactPoint);

         message.getFootstepDataList().add().set(footstepData);
      }

      RobotSide robotSide = numberOfSteps % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;

      FootstepDataMessage footstepData = new FootstepDataMessage();

      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint3D placeToStepInWorld = new FramePoint3D(soleFrame, 0.0, 0.0, 0.0);
      placeToStepInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      placeToStepInWorld.setX(0.3 * (numberOfSteps - 1));

      footstepData.getLocation().set(placeToStepInWorld);
      footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide.toByte());

      message.getFootstepDataList().add().set(footstepData);

      FramePose3D leftFootPose = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPostProcessingPacket postProcessingRequest = new FootstepPostProcessingPacket();
      postProcessingRequest.getFootstepDataList().set(message);
      postProcessingRequest.getLeftFootPositionInWorld().set(leftFootPose.getPosition());
      postProcessingRequest.getLeftFootOrientationInWorld().set(leftFootPose.getOrientation());
      postProcessingRequest.getRightFootPositionInWorld().set(rightFootPose.getPosition());
      postProcessingRequest.getRightFootOrientationInWorld().set(rightFootPose.getOrientation());
      for (Point2DReadOnly vertex : defaultSolePolygon.getVertexBufferView())
      {
         postProcessingRequest.getLeftFootContactPoints2d().add().set(vertex);
         postProcessingRequest.getRightFootContactPoints2d().add().set(vertex);
      }

      FootstepPostProcessingPacket postProcessingOutput = postProcessingModule.handleRequestPacket(postProcessingRequest);

      FootstepDataListMessage footstepDataListMessage = postProcessingOutput.getFootstepDataList();

      List<FootstepDataMessage> footsteps = new ArrayList<>(footstepDataListMessage.getFootstepDataList());

      int stepCounter = 0;
      for (RobotSide robotSide1 : RobotSide.values)
      {
         footStates.get(robotSide1).addVariableChangedListener(v -> {
            if (footStates.get(robotSide1).getEnumValue() == ConstraintType.SWING)
            {
               List<Point3D> contactPoints3D = footsteps.remove(stepCounter).getPredictedContactPoints2d();
               if (contactPoints3D.size() < 1)
               {
                  contactPointController.setNewContacts(defaultSolePolygon.getVertexBufferView(), robotSide1, true);
               }
               else
               {
                  List<Point2D> newContactPoints = contactPoints3D.stream().map(Point2D::new).collect(Collectors.toList());
                  contactPointController.setNewContacts(newContactPoints, robotSide1, true);
               }

            }
         });
      }

      drcSimulationTestHelper.publishToController(footstepDataListMessage);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions((swingDuration + transferDuration) * numberOfSteps + 5.0);
      assertTrue(success);
   }

   private static final double[] rightHandStraightSideJointAngles = new double[] {-0.5067668142160446, -0.3659876546358431, 1.7973796317575155, -1.2398714600960365, -0.005510224629709242, 0.6123343067479899, 0.12524505635696856};
   private static final double[] leftHandStraightSideJointAngles = new double[] {0.61130147334225, 0.22680071472282162, 1.6270339908033258, 1.2703560974484844, 0.10340544060719102, -0.6738299572358809, 0.13264785356924128};
   private static final SideDependentList<double[]> straightArmConfigs = new SideDependentList<>();
   static
   {
      straightArmConfigs.put(RobotSide.LEFT, leftHandStraightSideJointAngles);
      straightArmConfigs.put(RobotSide.RIGHT, rightHandStraightSideJointAngles);
   }

   private void armsUp() throws SimulationExceededMaximumTimeException
   {
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      // bring the arms in a stretched position
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
         armTrajectoryMessage.setRobotSide(robotSide.toByte());
         double[] armConfig = straightArmConfigs.get(robotSide);
         for (int i = 0; i < armConfig.length; i++)
         {
            TrajectoryPoint1DMessage trajectoryPoint = new TrajectoryPoint1DMessage();
            trajectoryPoint.setPosition(armConfig[i]);
            trajectoryPoint.setTime(0.5);
            OneDoFJointTrajectoryMessage jointTrajectory = new OneDoFJointTrajectoryMessage();
            jointTrajectory.getTrajectoryPoints().add().set(trajectoryPoint);
            armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(jointTrajectory);
         }
         drcSimulationTestHelper.publishToController(armTrajectoryMessage);
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
   }

   private static FootstepPlanningRequestPacket getRequest(FullHumanoidRobotModel fullRobotModel, PlanarRegionsList planarRegionsList, FramePose3D goalPose, FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      FramePose3D leftFoot = new FramePose3D(fullRobotModel.getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFoot = new FramePose3D(fullRobotModel.getSoleFrame(RobotSide.RIGHT));
      leftFoot.changeFrame(ReferenceFrame.getWorldFrame());
      rightFoot.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket request = new FootstepPlanningRequestPacket();
      request.setRequestedInitialStanceSide(FootstepPlanningRequestPacket.ROBOT_SIDE_LEFT);
      request.getStartLeftFootPose().set(leftFoot);
      request.getStartRightFootPose().set(rightFoot);

      SideDependentList<Pose3D> goalSteps = PlannerTools.createSquaredUpFootsteps(goalPose, footstepPlannerParameters.getIdealFootstepWidth());
      request.getGoalLeftFootPose().set(goalSteps.get(RobotSide.LEFT));
      request.getGoalRightFootPose().set(goalSteps.get(RobotSide.RIGHT));

      request.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      request.setPlanBodyPath(false);

      double timeout = 12.0;
      request.setTimeout(timeout);

      return request;
   }


   private void runTest(FootstepPlanningRequestPacket requestPacket) throws SimulationExceededMaximumTimeException
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      YoFramePoint3D goalPosition = new YoFramePoint3D("goalPosition", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition goalGraphic = new YoGraphicPosition("goalGraphic", goalPosition, 0.05, YoAppearance.Green());

      Pose3D goalMidFootPose = new Pose3D();
      goalMidFootPose.interpolate(requestPacket.getGoalLeftFootPose(), requestPacket.getGoalRightFootPose(), 0.5);
      goalPosition.set(goalMidFootPose.getPosition());
      yoGraphicsListRegistry.registerYoGraphic("Test", goalGraphic);
      drcSimulationTestHelper.addChildRegistry(registry);
      drcSimulationTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(yoGraphicsListRegistry);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setFromPacket(requestPacket);

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      footstepPlanningModule.getFootstepPlannerParameters().setMinimumFootholdPercent(0.99);
      footstepPlanningModule.getFootstepPlannerParameters().setMaximumStepZ(0.32);
      FootstepPlannerOutput plannerOutput = footstepPlanningModule.handleRequest(request);

      System.out.println("output. " + plannerOutput.getFootstepPlanningResult());

      if (!plannerOutput.getFootstepPlanningResult().validForExecution())
      {
         fail("Invalid footstep plan: " + plannerOutput.getFootstepPlanningResult());
      }

      FramePose3D leftFootPose = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPostProcessingPacket postProcessingRequest = new FootstepPostProcessingPacket();

      FootstepDataListMessage footstepDataListFromPlan = FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(),
                                                                                                                     -1.0,
                                                                                                                     -1.0);
      postProcessingRequest.getFootstepDataList().set(footstepDataListFromPlan);
      postProcessingRequest.getPlanarRegionsList().set(requestPacket.getPlanarRegionsListMessage());
      postProcessingRequest.getLeftFootPositionInWorld().set(leftFootPose.getPosition());
      postProcessingRequest.getLeftFootOrientationInWorld().set(leftFootPose.getOrientation());
      postProcessingRequest.getRightFootPositionInWorld().set(rightFootPose.getPosition());
      postProcessingRequest.getRightFootOrientationInWorld().set(rightFootPose.getOrientation());

      System.out.println("sending post processing request");

      FootstepPostProcessingPacket postProcessingOutputStatus = postProcessingModule.handleRequestPacket(postProcessingRequest);

      FootstepDataListMessage footstepDataListMessage = postProcessingOutputStatus.getFootstepDataList();
      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage message = footstepDataListMessage.getFootstepDataList().get(i);
         String outputMessage = "Step " + i + " Side = " + RobotSide.fromByte(message.getRobotSide());
         outputMessage += "\nPoint: " + message.getLocation() + ", Orientation: " + message.getOrientation();
         LogTools.info(outputMessage);
      }

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      double stepTime = footstepDataListMessage.getDefaultSwingDuration() + footstepDataListMessage.getDefaultTransferDuration();
      if (stepTime < 0.5)
      {
         WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
         stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      }
      double simulationTime = 2.0 + stepTime * footstepDataListMessage.getFootstepDataList().size();

      boolean success =  drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(goalMidFootPose.getPosition());
      center.addZ(0.7);

      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   private class ContactPointController implements RobotController
   {
      private List<? extends Point2DReadOnly> newContactPoints = null;
      private RobotSide robotSide = null;

      private AtomicBoolean setNewContactPoints = new AtomicBoolean(false);
      private boolean setOnStep = false;

      private SideDependentList<YoEnum<ConstraintType>> footStates;

      public ContactPointController(SideDependentList<YoEnum<ConstraintType>> footStates)
      {
         this.footStates = footStates;
      }

      /**
       * Changes the foot contact points of the robot.
       * The contact points can be changed immediately or when the foot is in swing.
       *
       * @param newContactPoints
       * @param robotSide
       * @param setOnStep
       */
      public void setNewContacts(List<? extends Point2DReadOnly> newContactPoints, RobotSide robotSide, boolean setOnStep)
      {
         if (setNewContactPoints.get())
         {
            System.err.println("New contact points are already waiting to be set.");
            return;
         }

         this.newContactPoints = newContactPoints;
         this.robotSide = robotSide;
         this.setOnStep = setOnStep;

         setNewContactPoints.set(true);
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return null;
      }

      @Override
      public String getName()
      {
         return null;
      }

      @Override
      public String getDescription()
      {
         return null;
      }

      @Override
      public void doControl()
      {
         if (setNewContactPoints.get())
         {
            if (!setOnStep)
               setNewContacts();
            else if (footStates.get(robotSide).getEnumValue() == ConstraintType.SWING)
               setNewContacts();
         }
      }

      private void setNewContacts()
      {
         String footJointName = drcSimulationTestHelper.getControllerFullRobotModel().getFoot(robotSide).getParentJoint().getName();
         HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

         int pointIndex = 0;
         List<GroundContactPoint> allGroundContactPoints = robot.getAllGroundContactPoints();

         for (GroundContactPoint point : allGroundContactPoints)
         {
            Joint parentJoint = point.getParentJoint();

            if (parentJoint.getName().equals(footJointName))
            {
               Point2DReadOnly newContactPoint = newContactPoints.get(pointIndex);

               point.setIsInContact(false);
               Vector3D offset = new Vector3D();
               point.getOffset(offset);

               offset.setX(newContactPoint.getX());
               offset.setY(newContactPoint.getY());

               point.setOffsetJoint(offset);
               pointIndex++;
            }
         }

//         if (footContactsInAnkleFrame != null)
//         {
//            footContactsInAnkleFrame.set(robotSide, newContactPoints);
//         }

         setNewContactPoints.set(false);
      }

   }

   private ArrayList<Point2D> generateContactPointsForRotatedLineOfContact(double angle, double xLine, double yLine)
   {
      double lineWidth = 0.01;

      // build default foot polygon:
      SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
      double footForwardOffset = steppingParameters.getFootForwardOffset();
      double footBackwardOffset = steppingParameters.getFootBackwardOffset();
      double footWidth = steppingParameters.getFootWidth();
      double toeWidth = steppingParameters.getToeWidth();

      ArrayList<Point2D> soleVertices = new ArrayList<Point2D>();
      soleVertices.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      soleVertices.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      ConvexPolygon2D solePolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(soleVertices));
      solePolygon.update();

      // shrink polygon and project line origin inside
      ConvexPolygon2D shrunkSolePolygon = new ConvexPolygon2D();
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      shrinker.scaleConvexPolygon(solePolygon, lineWidth/2.0 + (footWidth-toeWidth)/2.0, shrunkSolePolygon);

      Point2D lineOrigin = new Point2D(xLine, yLine);
      shrunkSolePolygon.orthogonalProjection(lineOrigin);

      // transform line and compute intersections with default foot polygon
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(angle);
      transform.getTranslation().set(lineOrigin.getX(), lineOrigin.getY(), 0.0);

      Line2D line = new Line2D(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));
      line.applyTransform(transform);

      line.shiftToLeft(lineWidth/2.0);
      Point2DBasics[] leftIntersections = solePolygon.intersectionWith(line);
      line.shiftToRight(lineWidth);
      Point2DBasics[] rightIntersections = solePolygon.intersectionWith(line);

      ArrayList<Point2D> ret = new ArrayList<Point2D>();
      ret.add(new Point2D(leftIntersections[0]));
      ret.add(new Point2D(leftIntersections[1]));
      ret.add(new Point2D(rightIntersections[0]));
      ret.add(new Point2D(rightIntersections[1]));
      return ret;
   }
}
