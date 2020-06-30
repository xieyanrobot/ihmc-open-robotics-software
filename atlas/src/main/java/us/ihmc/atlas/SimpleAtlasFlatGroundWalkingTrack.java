package us.ihmc.atlas;

import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasLegConfigurationParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleWalkingControllerStateFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

import java.io.InputStream;

public class SimpleAtlasFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {

      DRCRobotModel model = getModel();

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, model.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      new DRCFlatGroundWalkingTrack(robotInitialSetup,
                                    guiInitialSetup,
                                    scsInitialSetup,
                                    useVelocityAndHeadingScript,
                                    cheatWithGroundHeightAtForFootstep,
                                    model,
                                    new SimpleWalkingControllerStateFactory(model.getCapturePointPlannerParameters()));
   }

   private static DRCRobotModel getModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS,
                                 RobotTarget.SCS,
                                 false)
      {
         private static final String parameterFile = "/us/ihmc/atlas/parameters/experimental_controller_parameters.xml";

         @Override
         public String getParameterFileName()
         {
            return parameterFile;
         }

         @Override
         public InputStream getParameterOverwrites()
         {
            return null;
         }

         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return SimpleAtlasFlatGroundWalkingTrack.getWalkingControllerParameters(getTarget(), getJointMap(), getContactPointParameters());
         }
      };
   }

   private static WalkingControllerParameters getWalkingControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
   {
      return new AtlasWalkingControllerParameters(target, jointMap, contactPointParameters)
      {
         @Override
         public LegConfigurationParameters getLegConfigurationParameters()
         {
            return new AtlasLegConfigurationParameters(target == RobotTarget.REAL_ROBOT)
            {
               @Override
               public LegConfigurationGains getBentLegGains()
               {
                  LegConfigurationGains gains = super.getBentLegGains();
                  gains.setJointSpaceKp(200.0);
                  return gains;
               }
            };
         }
      };
   }
}
