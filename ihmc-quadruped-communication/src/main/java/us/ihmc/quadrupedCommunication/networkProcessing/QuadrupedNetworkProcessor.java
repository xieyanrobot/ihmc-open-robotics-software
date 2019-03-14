package us.ihmc.quadrupedCommunication.networkProcessing;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.bodyTeleop.QuadrupedBodyTeleopModule;
import us.ihmc.quadrupedCommunication.networkProcessing.footstepPlanning.QuadrupedFootstepPlanningModule;
import us.ihmc.quadrupedCommunication.networkProcessing.heightTeleop.QuadrupedBodyHeightTeleopModule;
import us.ihmc.quadrupedCommunication.networkProcessing.stepTeleop.QuadrupedStepTeleopModule;
import us.ihmc.quadrupedCommunication.networkProcessing.xBox.QuadrupedXBoxModule;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class QuadrupedNetworkProcessor
{
   private final boolean DEBUG = false;
   private QuadrupedStepTeleopModule stepTeleopModule;

   private final List<QuadrupedToolboxModule> modules = new ArrayList<>();

   public QuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel, QuadrupedNetworkModuleParameters params, double nominalHeight,
                                    FootstepPlannerParameters footstepPlannerParameters, QuadrupedXGaitSettings xGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters)
   {
      this(robotModel, params, nominalHeight, footstepPlannerParameters, xGaitSettings, pointFootSnapperParameters, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public QuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel, QuadrupedNetworkModuleParameters params, double nominalHeight,
                                    FootstepPlannerParameters footstepPlannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(robotModel, null, params, nominalHeight, footstepPlannerParameters, xGaitSettings, pointFootSnapperParameters, pubSubImplementation);
   }

   public QuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel, LogModelProvider logModelProvider, QuadrupedNetworkModuleParameters params,
                                    double nominalHeight, FootstepPlannerParameters footstepPlannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      tryToStartModule(() -> setupFootstepPlanningModule(robotModel, footstepPlannerParameters, xGaitSettings, pointFootSnapperParameters, logModelProvider,
                                                         params, pubSubImplementation));
      tryToStartModule(() -> setupStepTeleopModule(robotModel, xGaitSettings, pointFootSnapperParameters, logModelProvider, params, pubSubImplementation));
      tryToStartModule(() -> setupBodyHeightTeleopModule(robotModel, nominalHeight, logModelProvider, params, pubSubImplementation));
      tryToStartModule(() -> setupBodyTeleopModule(robotModel, logModelProvider, params, pubSubImplementation));
      tryToStartModule(() -> setupXBoxModule(robotModel, xGaitSettings, nominalHeight, logModelProvider, params, pubSubImplementation));
      tryToStartModule(() -> setupRobotEnvironmentAwarenessModule(params, pubSubImplementation));
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shift)
   {
      if (stepTeleopModule != null)
         stepTeleopModule.setShiftPlanBasedOnStepAdjustment(shift);
   }

   public void close()
   {
      for (int i = 0; i < modules.size(); i++)
      {
         modules.get(i).destroy();
      }
   }

   private void setupStepTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                      PointFootSnapperParameters pointFootSnapperParameters, LogModelProvider logModelProvider,
                                      QuadrupedNetworkModuleParameters params, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isStepTeleopModuleEnabled())
         return;
      stepTeleopModule = new QuadrupedStepTeleopModule(modelFactory, xGaitSettings, pointFootSnapperParameters, logModelProvider,
                                                       params.visualizeStepTeleopModuleEnabled(), params.logStepTeleopModuleEnabled(), pubSubImplementation);
      modules.add(stepTeleopModule);
   }

   private void setupFootstepPlanningModule(FullQuadrupedRobotModelFactory modelFactory, FootstepPlannerParameters footstepPlannerParameters,
                                            QuadrupedXGaitSettingsReadOnly xGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                            LogModelProvider logModelProvider, QuadrupedNetworkModuleParameters params,
                                            DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isFootstepPlanningModuleEnabled())
         return;
      modules.add(new QuadrupedFootstepPlanningModule(modelFactory, footstepPlannerParameters, xGaitSettings, pointFootSnapperParameters, logModelProvider,
                                                      params.visualizeFootstepPlanningModuleEnabled(), params.logFootstepPlanningModuleEnabled(),
                                                      pubSubImplementation));
   }


   private void setupBodyHeightTeleopModule(FullQuadrupedRobotModelFactory modelFactory, double nominalHeight, LogModelProvider logModelProvider,
                                            QuadrupedNetworkModuleParameters params, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isBodyHeightTeleopModuleEnabled())
         return;
      modules.add(new QuadrupedBodyHeightTeleopModule(modelFactory, nominalHeight, logModelProvider, params.visualizeBodyHeightTeleopModuleEnabled(),
                                                      params.logBodyHeightTeleopModuleEnabled(), pubSubImplementation));
   }

   private void setupBodyTeleopModule(FullQuadrupedRobotModelFactory modelFactory, LogModelProvider logModelProvider, QuadrupedNetworkModuleParameters params,
                                      DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (!params.isBodyTeleopModuleEnabled())
         return;
      modules.add(new QuadrupedBodyTeleopModule(modelFactory, logModelProvider, params.visualizeBodyTeleopModuleEnabled(),
                                                params.logBodyTeleopModuleEnabled(), pubSubImplementation));
   }

   private void setupXBoxModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double nominalBodyHeight,
                                LogModelProvider logModelProvider,  QuadrupedNetworkModuleParameters params,
                                DomainFactory.PubSubImplementation pubSubImplementation) throws IOException
   {
      if (!params.isXBoxModuleEnabled())
         return;
      modules.add(new QuadrupedXBoxModule(modelFactory, defaultXGaitSettings, nominalBodyHeight, logModelProvider, pubSubImplementation));
   }

   private void setupRobotEnvironmentAwarenessModule(QuadrupedNetworkModuleParameters params, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (params.isRobotEnvironmentAwarenessModuleEnabled())
      {
         try
         {
            if (pubSubImplementation == DomainFactory.PubSubImplementation.FAST_RTPS)
               LIDARBasedREAModule.createRemoteModule(System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt").start();
            else
               LIDARBasedREAModule.createIntraprocessModule(System.getProperty("user.home") + "/.ihmc/Configurations/defaultREAModuleConfiguration.txt")
                                  .start();

         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   protected void connect(PacketCommunicator communicator)
   {
      try
      {
         communicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void printModuleConnectedDebugStatement(PacketDestination destination, String methodName)
   {
      if (DEBUG)
      {
         PrintTools.debug(this, methodName + ": " + destination);
      }
   }

   private void tryToStartModule(ModuleStarter runnable)
   {
      try
      {
         runnable.startModule();
      }
      catch (RuntimeException | IOException e)
      {
         PrintTools.error(this, "Failed to start a module in the network processor, stack trace:");
         e.printStackTrace();
      }
   }

   private interface ModuleStarter
   {
      void startModule() throws IOException;
   }
}
