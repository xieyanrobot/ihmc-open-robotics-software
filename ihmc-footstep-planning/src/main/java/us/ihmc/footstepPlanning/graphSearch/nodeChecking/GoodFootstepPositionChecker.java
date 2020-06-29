package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.function.UnaryOperator;

public class GoodFootstepPositionChecker
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapAndWiggler snapper;

   private final TransformReferenceFrame startOfSwingFrame = new TransformReferenceFrame("startOfSwingFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame stanceFootFrame = new TransformReferenceFrame("stanceFootFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame candidateFootFrame = new TransformReferenceFrame("candidateFootFrame", ReferenceFrame.getWorldFrame());
   private final ZUpFrame startOfSwingZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), startOfSwingFrame, "startOfSwingZUpFrame");
   private final ZUpFrame stanceFootZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), stanceFootFrame, "stanceFootZUpFrame");
   private final FramePose3D stanceFootPose = new FramePose3D();
   private final FramePose3D candidateFootPose = new FramePose3D();

   private UnaryOperator<FootstepNode> parentNodeSupplier;

   private final YoEnum<BipedalFootstepPlannerNodeRejectionReason> rejectionReason = YoEnum.create("rejectionReason", "", BipedalFootstepPlannerNodeRejectionReason.class, registry, true);
   private final YoDouble stepWidth = new YoDouble("stepWidth", registry);
   private final YoDouble stepLength = new YoDouble("stepLength", registry);
   private final YoDouble stepHeight = new YoDouble("stepHeight", registry);
   private final YoDouble stepReachXY = new YoDouble("stepReachXY", registry);

   public GoodFootstepPositionChecker(FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapAndWiggler snapper, YoVariableRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      parentRegistry.addChild(registry);
   }

   public void setParentNodeSupplier(UnaryOperator<FootstepNode> parentNodeSupplier)
   {
      this.parentNodeSupplier = parentNodeSupplier;
   }

   public boolean isNodeValid(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      RobotSide stepSide = candidateNode.getRobotSide();

      FootstepNodeSnapData candidateNodeSnapData = snapper.snapFootstepNode(candidateNode);
      FootstepNodeSnapData stanceNodeSnapData = snapper.snapFootstepNode(stanceNode);

      candidateFootFrame.setTransformAndUpdate(candidateNodeSnapData.getSnappedNodeTransform(candidateNode));
      stanceFootFrame.setTransformAndUpdate(stanceNodeSnapData.getSnappedNodeTransform(stanceNode));
      stanceFootZUpFrame.update();

      candidateFootPose.setToZero(candidateFootFrame);
      candidateFootPose.changeFrame(stanceFootZUpFrame);

      stanceFootPose.setToZero(stanceFootFrame);
      stanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      stepLength.set(candidateFootPose.getX());
      stepWidth.set(stepSide.negateIfRightSide(candidateFootPose.getY()));
      stepReachXY.set(EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(candidateFootPose.getX()), Math.abs(stepWidth.getValue() - parameters.getIdealFootstepWidth())));
      stepHeight.set(candidateFootPose.getZ());

      if (stepWidth.getValue() < parameters.getMinimumStepWidth())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH);
         return false;
      }
      else if (stepWidth.getValue() > parameters.getMaximumStepWidth())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE);
         return false;
      }
      else if (stepLength.getValue() < parameters.getMinimumStepLength())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH);
         return false;
      }
      else if (Math.abs(stepHeight.getValue()) > parameters.getMaximumStepZ())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW);
         return false;
      }

      double alphaPitchedBack = Math.max(0.0, - stanceFootPose.getPitch() / parameters.getMinimumSurfaceInclineRadians());
      double minZFromPitchContraint = InterpolationTools.linearInterpolate(Math.abs(parameters.getMaximumStepZ()), Math.abs(parameters.getMinimumStepZWhenFullyPitched()), alphaPitchedBack);
      double maxXFromPitchContraint = InterpolationTools.linearInterpolate(Math.abs(parameters.getMaximumStepReach()), parameters.getMaximumStepXWhenFullyPitched(), alphaPitchedBack);
      double stepDownFraction = - stepHeight.getValue() / minZFromPitchContraint;
      double stepForwardFraction = stepLength.getValue() / maxXFromPitchContraint;

      boolean stepIsPitchedBack = alphaPitchedBack > 0.0;
      boolean stepTooLow = stepLength.getValue() > 0.0 && stepDownFraction > 1.0;
      boolean stepTooForward = stepHeight.getValue() < 0.0 && stepForwardFraction > 1.0;

      if (stepIsPitchedBack && (stepTooLow || stepTooForward))
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_LOW_AND_FORWARD_WHEN_PITCHED);
         return false;
      }

      double maxReach = parameters.getMaximumStepReach();
      if (stepHeight.getValue() < -Math.abs(parameters.getMaximumStepZWhenForwardAndDown()))
      {
         if (stepLength.getValue() > parameters.getMaximumStepXWhenForwardAndDown())
         {
            rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FORWARD_AND_DOWN);
            return false;
         }

         if (stepWidth.getValue() > parameters.getMaximumStepYWhenForwardAndDown())
         {
            rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE_AND_DOWN);
            return false;
         }

         maxReach = EuclidCoreTools.norm(parameters.getMaximumStepXWhenForwardAndDown(), parameters.getMaximumStepYWhenForwardAndDown() - parameters.getIdealFootstepWidth());
      }

      if (stepReachXY.getValue() > parameters.getMaximumStepReach())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR);
         return false;
      }

      if (stepHeight.getValue() > parameters.getMaximumStepZWhenSteppingUp())
      {
         if (stepReachXY.getValue() > parameters.getMaximumStepReachWhenSteppingUp())
         {
            rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_AND_HIGH);
            return false;
         }
         if (stepWidth.getValue() > parameters.getMaximumStepWidthWhenSteppingUp())
         {
            rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE_AND_HIGH);
            return false;
         }

         maxReach = parameters.getMaximumStepReachWhenSteppingUp();
      }

      double stepReach3D = EuclidCoreTools.norm(stepReachXY.getValue(), stepHeight.getValue());
      double maxInterpolationFactor = Math.max(stepReach3D / maxReach, Math.abs(stepHeight.getValue() / parameters.getMaximumStepZ()));
      maxInterpolationFactor = Math.min(maxInterpolationFactor, 1.0);
      double maxYaw = InterpolationTools.linearInterpolate(parameters.getMaximumStepYaw(), (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMaximumStepYaw(),
                                                           maxInterpolationFactor);
      double minYaw = InterpolationTools.linearInterpolate(parameters.getMinimumStepYaw(), (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMinimumStepYaw(),
                                                           maxInterpolationFactor);
      double yawDelta = AngleTools.computeAngleDifferenceMinusPiToPi(candidateNode.getYaw(), stanceNode.getYaw());
      if (!MathTools.intervalContains(stepSide.negateIfRightSide(yawDelta), minYaw, maxYaw))
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH);
         return false;
      }

      // Check reach from start of swing
      FootstepNode grandParentNode;
      FootstepNodeSnapData grandparentNodeSnapData;
      double alphaSoS = parameters.getTranslationScaleFromGrandparentNode();
      if (alphaSoS > 0.0 && parentNodeSupplier != null && (grandParentNode = parentNodeSupplier.apply(stanceNode)) != null
          && (grandparentNodeSnapData = snapper.snapFootstepNode(grandParentNode)) != null)
      {
         startOfSwingFrame.setTransformAndUpdate(grandparentNodeSnapData.getSnappedNodeTransform(grandParentNode));
         startOfSwingZUpFrame.update();
         candidateFootPose.changeFrame(startOfSwingZUpFrame);
         double swingHeight = candidateFootPose.getZ();
         double swingReach = EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(candidateFootPose.getX()), Math.abs(candidateFootPose.getY()));

         if (swingHeight > parameters.getMaximumStepZWhenSteppingUp())
         {
            if (swingReach > alphaSoS * parameters.getMaximumStepReachWhenSteppingUp())
            {
               rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR_AND_HIGH);
               return false;
            }
         }
      }

      return true;
   }

   void clearLoggedVariables()
   {
      stepWidth.setToNaN();
      stepLength.setToNaN();
      stepHeight.setToNaN();
      stepReachXY.setToNaN();
   }

   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason.getEnumValue();
   }
}
