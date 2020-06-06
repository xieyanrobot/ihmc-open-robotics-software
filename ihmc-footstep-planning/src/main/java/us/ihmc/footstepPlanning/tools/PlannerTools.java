package us.ihmc.footstepPlanning.tools;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class PlannerTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final double footLength = 0.2;
   public static final double footWidth = 0.1;

   public static ConvexPolygon2D createFootPolygon(double footLength, double heelWidth, double toeWidth)
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(footLength / 2.0, toeWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -toeWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, heelWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -heelWidth / 2.0);
      footPolygon.update();

      return footPolygon;
   }

   public static ConvexPolygon2D createFootPolygon(double footLength, double footWidth)
   {
      return createFootPolygon(footLength, footWidth, footWidth);
   }

   public static ConvexPolygon2D createDefaultFootPolygon()
   {
      return createFootPolygon(footLength, footWidth);
   }

   public static SideDependentList<ConvexPolygon2D> createDefaultFootPolygons()
   {
      return createFootPolygons(footLength, footWidth);
   }

   public static SideDependentList<ConvexPolygon2D> createFootPolygons(double footLength, double footWidth)
   {
      return createFootPolygons(footLength, footWidth, footWidth);
   }

   public static SideDependentList<ConvexPolygon2D> createFootPolygons(double footLength, double heelWidth, double toeWidth)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.put(side, createFootPolygon(footLength, heelWidth, toeWidth));
      return footPolygons;
   }

   public static SideDependentList<Pose3D> createSquaredUpFootsteps(Point3DReadOnly midFootPosition, double midFootYaw, double stanceWidth)
   {
      Pose3D midFootPose = new Pose3D(midFootPosition, new Quaternion(midFootYaw, 0.0, 0.0));
      return createSquaredUpFootsteps(midFootPose, stanceWidth);
   }

   public static SideDependentList<Pose3D> createSquaredUpFootsteps(Pose3DReadOnly midFootPose, double stanceWidth)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3D footstepPose = new Pose3D(midFootPose);
                                        footstepPose.appendTranslation(0.0, 0.5 * side.negateIfRightSide(stanceWidth), 0.0);
                                        return footstepPose;
                                     });
   }

   public static void addGoalViz(FramePose3D goalPose, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      YoFramePoint3D yoGoal = new YoFramePoint3D("GoalPosition", worldFrame, registry);
      yoGoal.set(goalPose.getPosition());
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("GoalViz", yoGoal, 0.05, YoAppearance.Yellow()));
      YoFramePoint3D yoStart = new YoFramePoint3D("StartPosition", worldFrame, registry);
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("StartViz", yoStart, 0.05, YoAppearance.Blue()));
      PoseReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
      FrameVector3D goalOrientation = new FrameVector3D(goalFrame, 0.5, 0.0, 0.0);
      goalOrientation.changeFrame(worldFrame);
      YoFrameVector3D yoGoalOrientation = new YoFrameVector3D("GoalVector", worldFrame, registry);
      yoGoalOrientation.set(goalOrientation);
//      graphicsListRegistry.registerYoGraphic("vizOrientation", new YoGraphicVector("GoalOrientationViz", yoGoal, yoGoalOrientation, 1.0, YoAppearance.White()));
   }

   public static boolean isGoalNextToLastStep(FramePose3D goalPose, FootstepPlan footstepPlan)
   {
      return isGoalNextToLastStep(goalPose, footstepPlan, 0.5);
   }

   public static boolean isGoalNextToLastStep(FramePose3D goalPose, FootstepPlan footstepPlan, double epsilon)
   {
      int steps = footstepPlan.getNumberOfSteps();
      if (steps < 1)
         throw new RuntimeException("Did not get enough footsteps to check if goal is within feet.");

      Footstep footstep = footstepPlan.getFootstep(steps - 1);
      FramePose3D stepPose = new FramePose3D();
      footstep.getPose(stepPose);
      RobotSide stepSide = footstep.getRobotSide();

      double midFeetOffset = stepSide.negateIfLeftSide(0.125);
      Vector3D goalOffset = new Vector3D(0.0, midFeetOffset, 0.0);
      RigidBodyTransform soleToWorld = new RigidBodyTransform();
      stepPose.get(soleToWorld);
      soleToWorld.transform(goalOffset);

      FramePose3D achievedGoal = new FramePose3D(stepPose);
      Point3D goalPosition = new Point3D(achievedGoal.getPosition());
      goalPosition.add(goalOffset);
      achievedGoal.getPosition().set(goalPosition);

      if (achievedGoal.epsilonEquals(goalPose, epsilon))
         return true;
      else
         return false;
   }

   public static boolean isGoalNextToLastStep(Point3D goalPosition, FootstepPlan footstepPlan)
   {
      return isGoalNextToLastStep(goalPosition, footstepPlan, 0.5);
   }

   public static boolean isGoalNextToLastStep(Point3D desiredPosition, FootstepPlan footstepPlan, double epsilon)
   {
      Point3D goalPosition = getEndPosition(footstepPlan);

      if (goalPosition.epsilonEquals(desiredPosition, epsilon))
         return true;
      else
         return false;
   }

   public static Point3D getEndPosition(FootstepPlan footstepPlan)
   {
      int steps = footstepPlan.getNumberOfSteps();
      if (steps < 1)
         throw new RuntimeException("Did not get enough footsteps to get end position.");

      Footstep footstep = footstepPlan.getFootstep(steps - 1);
      FramePose3D stepPose = new FramePose3D();
      footstep.getPose(stepPose);
      RobotSide stepSide = footstep.getRobotSide();

      double midFeetOffset = stepSide.negateIfLeftSide(0.125);
      Vector3D goalOffset = new Vector3D(0.0, midFeetOffset, 0.0);
      RigidBodyTransform soleToWorld = new RigidBodyTransform();
      stepPose.get(soleToWorld);
      soleToWorld.transform(goalOffset);

      FramePose3D achievedGoal = new FramePose3D(stepPose);
      Point3D goalPosition = new Point3D(achievedGoal.getPosition());
      goalPosition.add(goalOffset);

      return goalPosition;
   }
}
