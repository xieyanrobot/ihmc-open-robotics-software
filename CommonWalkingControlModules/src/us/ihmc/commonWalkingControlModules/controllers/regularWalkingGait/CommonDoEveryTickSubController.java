package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFeetUpdater;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturePointCalculatorInterface;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.DoubleAndSingleSupportDurationUpdater;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class CommonDoEveryTickSubController implements DoEveryTickSubController
{
   private final CommonWalkingReferenceFrames referenceFrames;

   private final BipedFootInterface leftFoot;
   private final BipedFootInterface rightFoot;
   private final BipedFeetUpdater bipedFeetUpdater;

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final DoubleAndSingleSupportDurationUpdater doubleAndSingleSupportDurationUpdater;

   private final CapturePointCalculatorInterface capturePointCalculator;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final CouplingRegistry couplingRegistry;

   private ArrayList<Updatable> updatables;

   private double initialDesiredHeading;
   private final ProcessedSensorsInterface processedSensors;

   private boolean recycleBipedFeetMemory = false;

   public CommonDoEveryTickSubController(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames, BipedFootInterface leftFoot,
         BipedFootInterface rightFoot, BipedFeetUpdater bipedFeetUpdater, Updatable footPolygonVisualizer,
         DesiredHeadingControlModule desiredHeadingControlModule, DesiredVelocityControlModule desiredVelocityControlModule,
         DesiredFootstepCalculator desiredFootstepCalculator, DoubleAndSingleSupportDurationUpdater doubleAndSingleSupportDurationUpdater,
         CapturePointCalculatorInterface capturePointCalculator, OneStepCaptureRegionCalculator captureRegionCalculator, CouplingRegistry couplingRegistry,
         double initialDesiredHeading)
   {
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;

      this.leftFoot = leftFoot;
      this.rightFoot = rightFoot;
      this.bipedFeetUpdater = bipedFeetUpdater;

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.doubleAndSingleSupportDurationUpdater = doubleAndSingleSupportDurationUpdater;

      this.capturePointCalculator = capturePointCalculator;
      this.captureRegionCalculator = captureRegionCalculator;
      this.couplingRegistry = couplingRegistry;
      this.initialDesiredHeading = initialDesiredHeading;

      addUpdatable(footPolygonVisualizer);
   }

   public void addUpdatable(Updatable updatable)
   {
      if (updatables == null)
      {
         updatables = new ArrayList<Updatable>();
      }

      updatables.add(updatable);
   }

   private void doUpdatables(double time)
   {
      if (updatables != null)
      {
         for (Updatable updatable : updatables)
         {
            updatable.update(time);
         }
      }
   }

   public void doEveryControlTick(RobotSide supportLeg)
   {
      desiredVelocityControlModule.updateDesiredVelocity();
      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();
      couplingRegistry.setDesiredVelocity(desiredVelocity);
      desiredHeadingControlModule.updateDesiredHeadingFrame();

      // Compute the instantaneous capture point.
      capturePointCalculator.computeCapturePoint(supportLeg);
      FramePoint capturePointInMidfeetZUp = capturePointCalculator.getCapturePointInFrame(referenceFrames.getMidFeetZUpFrame());
      couplingRegistry.setCapturePoint(capturePointInMidfeetZUp);

      boolean forceHindOnToes = couplingRegistry.getForceHindOnToes();
      bipedFeetUpdater.updateBipedFeet(leftFoot, rightFoot, supportLeg, capturePointInMidfeetZUp, forceHindOnToes);

      BipedSupportPolygons bipedSupportPolygons = couplingRegistry.getBipedSupportPolygons();

      bipedSupportPolygons.update(new SideDependentList<List<FramePoint>>(leftFoot.computeFootPoints(), rightFoot.computeFootPoints()));
      doUpdatables(processedSensors.getTime());

      if (supportLeg != null)
      {
         // TODO: also compute capture regions in double support
         FramePoint2d capturePoint2d = capturePointInMidfeetZUp.toFramePoint2d();
         FrameConvexPolygon2d supportFoot = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
         double gravity = -processedSensors.getGravityInWorldFrame().getZ();
         double comHeight = processedSensors.getCenterOfMassPositionInFrame(referenceFrames.getMidFeetZUpFrame()).getZ();
         double omega0 = Math.sqrt(gravity / comHeight);
         captureRegionCalculator.calculateCaptureRegion(supportLeg.getOppositeSide(), couplingRegistry.getEstimatedSwingTimeRemaining(), capturePoint2d,
               omega0, supportFoot);
         FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
         couplingRegistry.setCaptureRegion(captureRegion);

         // Desired Footstep
         Footstep desiredFootstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);

         couplingRegistry.setDesiredFootstep(desiredFootstep);

         doubleAndSingleSupportDurationUpdater.update(desiredFootstep, supportLeg, desiredVelocity);
         couplingRegistry.setDoubleSupportDuration(doubleAndSingleSupportDurationUpdater.getDoubleSupportDuration());
         couplingRegistry.setSingleSupportDuration(doubleAndSingleSupportDurationUpdater.getSingleSupportDuration());
      }
      else
      {
         captureRegionCalculator.hideCaptureRegion();
         couplingRegistry.setCaptureRegion(null);
         couplingRegistry.setDesiredFootstep(null);
      }
   }

   public void initialize()
   {
      if (Double.isNaN(initialDesiredHeading))
      {
         System.out.println("Resetting desired heading to current heading.");
         initialDesiredHeading = processedSensors.getPelvisOrientationInFrame(ReferenceFrame.getWorldFrame()).getYawPitchRoll()[0];
      }

      System.out.println("Resetting desired heading to " + initialDesiredHeading);
      desiredHeadingControlModule.resetHeadingAngle(initialDesiredHeading);
   }
}