package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.AbstractICPPlanner;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class SmoothCMPBasedICPPlanner extends AbstractICPPlanner
{
   private static final boolean VISUALIZE = true;

   private final ReferenceCoPTrajectoryGenerator referenceCoPGenerator;
   private final ReferenceCMPTrajectoryGenerator referenceCMPGenerator;
   private final ReferenceICPTrajectoryGenerator referenceICPGenerator;

   private final List<YoDouble> swingDurationShiftFractions = new ArrayList<>();

   public SmoothCMPBasedICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                   int maxNumberOfFootstepsToConsider, int numberOfPointsPerFoot, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, maxNumberOfFootstepsToConsider);

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         YoDouble swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + i, registry);
         swingDurationShiftFractions.add(swingDurationShiftFraction);
      }

      referenceCoPGenerator = new ReferenceCoPTrajectoryGenerator(namePrefix, numberOfPointsPerFoot, maxNumberOfFootstepsToConsider, bipedSupportPolygons, contactableFeet,
                                                                  numberFootstepsToConsider, swingDurations, transferDurations, swingDurationAlphas,
                                                                  swingDurationShiftFractions, transferDurationAlphas,
                                                                  registry);

      referenceCMPGenerator = new ReferenceCMPTrajectoryGenerator(namePrefix, maxNumberOfFootstepsToConsider, numberFootstepsToConsider, swingDurations, transferDurations,
                                                                  swingDurationAlphas, transferDurationAlphas, registry);

      referenceICPGenerator = new ReferenceICPTrajectoryGenerator(namePrefix, omega0, numberFootstepsToConsider, isStanding, useDecoupled, worldFrame,
                                                                  registry);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }
   }

   public void initializeParameters(ICPPlannerParameters icpPlannerParameters)
   {
      super.initializeParameters((ICPTrajectoryPlannerParameters) icpPlannerParameters);

      if (icpPlannerParameters instanceof SmoothCMPPlannerParameters)
      {
         numberFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());

         referenceCoPGenerator.initializeParameters((SmoothCMPPlannerParameters) icpPlannerParameters);

         for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
         {
            swingDurationShiftFractions.get(i).set(((SmoothCMPPlannerParameters) icpPlannerParameters).getSwingDurationShiftFraction());
         }
      }
      else
      {
         throw new RuntimeException("Tried to submit the wrong type of parameters.");
      }
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      referenceCoPGenerator.createVisualizerForConstantCoPs(yoGraphicsList, artifactList);

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   /** {@inheritDoc} */
   public void clearPlan()
   {
      referenceCoPGenerator.clear();
      referenceCMPGenerator.reset();
      referenceICPGenerator.reset();

      for (int i = 0; i < swingDurations.size(); i++)
      {
         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();
         swingDurationAlphas.get(i).setToNaN();
         transferDurationAlphas.get(i).setToNaN();
      }
   }

   @Override
   /** {@inheritDoc} */
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      referenceCoPGenerator.addFootstepToPlan(footstep, timing);
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForStanding(double initialTime)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForTransfer(double initialTime)
   {
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);

      //referenceCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport();
      referenceCMPGenerator.initializeForTransfer(initialTime, referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInTransfer()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public void initializeForSingleSupport(double initialTime)
   {
      referenceCMPGenerator.initializeForSwing(initialTime, referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInSwing()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   protected void updateTransferPlan()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   protected void updateSingleSupportPlan()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public void compute(double time)
   {
      throw new RuntimeException("to implement");
   }


   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCenterOfMassPosition(FramePoint2d finalDesiredCenterOfMassPositionToPack)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public boolean isOnExitCMP()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsToConsider()
   {
      throw new RuntimeException("to implement");
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsRegistered()
   {
      throw new RuntimeException("to implement");
   }
}
