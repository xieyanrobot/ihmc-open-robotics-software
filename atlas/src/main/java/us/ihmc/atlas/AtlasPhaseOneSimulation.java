package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.robotEnvironmentAwareness.tools.ConstantPlanarRegionsPublisher;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.Random;

public class AtlasPhaseOneSimulation
{
   public AtlasPhaseOneSimulation()
   {
//      PlanarRegionsList startingRegions = PlanarRegionFileTools.importPlanarRegionData(new File(
//            System.getProperty("user.home") + File.separator + "PlanarRegions_Start"));

      RigidBodyTransform startingBlockTransform = new RigidBodyTransform();
      startingBlockTransform.getTranslation().set(-2.0, -1.0, 0.0);
      startingBlockTransform.getRotation().setYawPitchRoll(Math.toRadians(20.0), 0.0, 0.0);
      PlanarRegionsList startingRegions = generateStartingBlockRegions(startingBlockTransform);

      PlanarRegionsList middleRegions = BehaviorPlanarRegionEnvironments.createRoughUpAndDownStairsWithFlatTop();

      PlanarRegionsList planarRegionsList = new PlanarRegionsList();

      for (int i = 0; i < startingRegions.getNumberOfPlanarRegions(); i++)
      {
         planarRegionsList.addPlanarRegion(startingRegions.getPlanarRegion(i));
      }

      for (int i = 0; i < middleRegions.getNumberOfPlanarRegions(); i++)
      {
         planarRegionsList.addPlanarRegion(middleRegions.getPlanarRegion(i));
      }

      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints, false, false);

      Pose3D startingPose = new Pose3D();
      startingPose.getPosition().set(-2.0, -1.0, 0.3);
      startingPose.getOrientation().setYawPitchRoll(Math.toRadians(20.0), 0.0, 0.0);

      PlanarRegionsListDefinedEnvironment simEnvironment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.02, true);
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, simEnvironment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setStartingLocation(() -> new OffsetAndYawRobotInitialSetup(startingPose.getPosition(), startingPose.getYaw()));
      HumanoidNetworkProcessorParameters networkProcessorParameters = new HumanoidNetworkProcessorParameters();

      // talk to controller and footstep planner
      networkProcessorParameters.setUseFootstepPlanningToolboxModule(false);
      networkProcessorParameters.setUseWalkingPreviewModule(true);
      networkProcessorParameters.setUseBipedalSupportPlanarRegionPublisherModule(true);

      // disable everything else
      networkProcessorParameters.setUseSensorModule(true);
      networkProcessorParameters.setUseHumanoidAvatarREAStateUpdater(true);

      // start sim
      simulationStarter.startSimulation(networkProcessorParameters, false);

      // spoof and publish planar regions
      ConstantPlanarRegionsPublisher constantPlanarRegionsPublisher = new ConstantPlanarRegionsPublisher(planarRegionsList);
      constantPlanarRegionsPublisher.start(2000);
   }

   private static PlanarRegionsList generateStartingBlockRegions(RigidBodyTransform startingPose)
   {
      double platformXY = 1.0;
      double platformHeight = 0.3;

      double blockXY = 0.35;
      double blockHeight = 0.15;
      double blockSpacing = 0.1;

      double maxBlockAngle = Math.toRadians(20.0);
      double lastRowMaxBlockAngle = Math.toRadians(10.0);

      Random random = new Random(328903);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setTransform(startingPose);
      generator.setId(250);
      generator.addCubeReferencedAtBottomMiddle(platformXY, platformXY, platformHeight);
      generator.translate(0.5 * (platformXY + blockXY), 3.0 * (blockXY + blockSpacing), 0.0);

      for (int i = 0; i < 5; i++)
      {
         double height = platformHeight - blockHeight - 0.05 * (i + 1);
         generator.translate(0.0, -4.0 * (blockXY + blockSpacing), 0.0);

         if (i % 2 == 0)
         {
            generator.translate(0.0, -0.5 * (blockXY + blockSpacing), 0.0);
         }
         else
         {
            generator.translate(0.0, 0.5 * (blockXY + blockSpacing), 0.0);
         }

         for (int j = 0; j < 4; j++)
         {
            double rotationAxisDirection = 2.0 * Math.PI * random.nextDouble();
            double rotationAngle = (i == 4 ? lastRowMaxBlockAngle : maxBlockAngle) * random.nextDouble();
            double yawRotation = 2.0 * Math.PI * random.nextDouble();

            Vector3D rotationAxis = new Vector3D(Math.cos(rotationAxisDirection), Math.sin(rotationAxisDirection), 0.0);
            rotationAxis.scale(rotationAngle);
            AxisAngle axisAngle = new AxisAngle(rotationAxis, rotationAngle);
            RotationMatrix rotationMatrix = new RotationMatrix();
            axisAngle.get(rotationMatrix);

            generator.translate(0.0, 0.0, height);
            generator.rotate(rotationMatrix);
            generator.rotate(yawRotation, Axis3D.Z);
            generator.addCubeReferencedAtBottomMiddle(blockXY, blockXY, blockHeight);
            generator.rotate(-yawRotation, Axis3D.Z);
            rotationMatrix.invert();
            generator.rotate(rotationMatrix);
            generator.translate(0.0, 0.0, -height);

            generator.translate(0.0, blockXY + blockSpacing, 0.0);
         }

         generator.translate(blockXY, 0.0, 0.0);
      }

      return generator.getPlanarRegionsList();
   }

   public static void main(String[] args)
   {
      new AtlasPhaseOneSimulation();
   }
}
