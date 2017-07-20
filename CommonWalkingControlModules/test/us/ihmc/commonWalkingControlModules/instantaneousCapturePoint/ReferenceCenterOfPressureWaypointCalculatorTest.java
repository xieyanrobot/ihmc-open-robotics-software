package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.ReferenceCoPTrajectoryGenerator;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceCenterOfPressureWaypointCalculatorTest
{
   private final int numberOfContactPoints = 4;
   private final double soleFrameYDisplacement = 0.2;
   private final double ankleFrameZDisplacement = 0.05;
   private final double frontContactX = 0.07;
   private final double rearContactX = -0.07;
   private final double side1ContactY = 0.03;
   private final double side2ContactY = -0.03;
   private final double swingTime = 1;
   private final double transferTime = 0.1;
   private final double stepLength = 0.3;
   private final double stepWidth = soleFrameYDisplacement;
   private final double EPSILON = 10e-5;

   ReferenceCoPTrajectoryGenerator testCoPGenerator;
   SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   SideDependentList<ContactableFoot> contactableFeet = new SideDependentList<>();
   SideDependentList<RigidBody> feetBodies = new SideDependentList<>();
   YoVariableRegistry parentRegistry = new YoVariableRegistry("TestRegistry");
   MidFootZUpGroundFrame midFeetZUpFrame;
   SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   SmoothCMPPlannerParameters plannerParameters;
   YoInteger numberOfFootstepsToConsider = new YoInteger("numberOfFootstepsToConsider", parentRegistry);
   private final ArrayList<YoDouble> swingDurations = new ArrayList<>();
   private final ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();
   private final ArrayList<YoDouble> swingDurationShiftFractions = new ArrayList<>();
   private final ArrayList<YoDouble> transferDurations = new ArrayList<>();
   private final ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();

   @Before
   public void setUp()
   {
      for (RobotSide side : RobotSide.values)
      {
         ZUpFrame soleFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(),
                                           new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, side.negateIfRightSide(soleFrameYDisplacement), 0.0),
                                           "DummyRobot" + side.toString() + "FootSoleFrame");
         soleZUpFrames.put(side, soleFrame);
         ZUpFrame ankleFrame = new ZUpFrame(soleFrame, new FramePoint(soleFrame, 0.0, 0.0, ankleFrameZDisplacement),
                                            "DummyRobot" + side.toString() + "AnkleSoleFrame");
         ankleZUpFrames.put(side, ankleFrame);
         List<Point2D> contactPoints = new ArrayList<>(numberOfContactPoints);
         Point2D point1 = new Point2D(frontContactX, side.negateIfRightSide(side1ContactY));
         contactPoints.add(point1);
         Point2D point2 = new Point2D(frontContactX, side.negateIfRightSide(side2ContactY));
         contactPoints.add(point2);
         Point2D point3 = new Point2D(rearContactX, side.negateIfRightSide(side1ContactY));
         contactPoints.add(point3);
         Point2D point4 = new Point2D(rearContactX, side.negateIfRightSide(side2ContactY));
         contactPoints.add(point4);
         ContactableFoot contactableFoot = new ListOfPointsContactableFoot(null, (ReferenceFrame) soleFrame, contactPoints, point1,
                                                                           new LineSegment2D(point1, point2));
         contactableFeet.put(side, contactableFoot);
         RigidBody feetBody = new RigidBody(side.toString() + "Feet", ReferenceFrame.getWorldFrame());
         feetBodies.put(side, feetBody);
         List<FramePoint2d> contactFramePoints = new ArrayList<>();
         contactFramePoints.add(new FramePoint2d(soleFrame, point1));
         contactFramePoints.add(new FramePoint2d(soleFrame, point2));
         contactFramePoints.add(new FramePoint2d(soleFrame, point3));
         contactFramePoints.add(new FramePoint2d(soleFrame, point4));

         YoPlaneContactState contactState = new YoPlaneContactState("DummyRobot" + side.toString() + "FootContactState", feetBody, soleFrame,
                                                                    contactFramePoints, 10, parentRegistry);
         contactStates.put(side, contactState);
      }
      midFeetZUpFrame = new MidFootZUpGroundFrame("DummyRobotMidFootZUpFrame", soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, parentRegistry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);
      plannerParameters = new SmoothCMPPlannerParameters();
      numberOfFootstepsToConsider.set(plannerParameters.getNumberOfFootstepsToConsider());

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         YoDouble swingDuration = new YoDouble("swingDuration" + i, parentRegistry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, parentRegistry);
         YoDouble swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + i, parentRegistry);
         YoDouble transferDuration = new YoDouble("transferDuration" + i, parentRegistry);
         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, parentRegistry);

         swingDurations.add(swingDuration);
         swingSplitFractions.add(swingSplitFraction);
         swingDurationShiftFractions.add(swingDurationShiftFraction);
         transferDurations.add(transferDuration);
         transferSplitFractions.add(transferSplitFraction);
      }

      int numberOfPointsInFoot = plannerParameters.getNumberOfCoPWayPointsPerFoot();
      int maxNumberOfFootstepsToConsider = plannerParameters.getNumberOfFootstepsToConsider();
      testCoPGenerator = new ReferenceCoPTrajectoryGenerator("TestCoPPlanClass", numberOfPointsInFoot, maxNumberOfFootstepsToConsider, bipedSupportPolygons,
                                                             contactableFeet, numberOfFootstepsToConsider, swingDurations, transferDurations,
                                                             swingSplitFractions, swingDurationShiftFractions, transferSplitFractions, parentRegistry);
      testCoPGenerator.initializeParameters(plannerParameters);
      assertTrue("Object not initialized", testCoPGenerator != null);
   }

   @After
   public void clearAllVariables()
   {
      testCoPGenerator = null;
      soleZUpFrames.clear();
      ankleZUpFrames.clear();
      contactableFeet.clear();
      feetBodies.clear();
      midFeetZUpFrame = null;
      contactStates.clear();
      plannerParameters = null;
   }
   
   public void sendFootStepMessages(int numberOfFootstepsToPlan)
   {
      RobotSide robotSide = RobotSide.LEFT;
      FramePoint footstepLocation = new FramePoint();
      FrameOrientation footstepOrientation = new FrameOrientation();
      for (int i = 1; i < numberOfFootstepsToPlan + 1; i++)
      {
         Footstep footstep = new Footstep(feetBodies.get(robotSide), robotSide);
         footstepLocation.set(i * stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);
         footstep.setPose(footstepLocation, footstepOrientation);
         FootstepTiming timing = new FootstepTiming(swingTime, transferTime);
         testCoPGenerator.addFootstepToPlan(footstep, timing);
         robotSide = robotSide.getOppositeSide();
      }
   }

   @Test
   public void testDoubleSupportFootstepPlanFromRest()
   {
      int numberOfFootsteps = 3;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepsRegistered() == numberOfFootsteps);
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(true, RobotSide.RIGHT);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();
      // initial waypoint between the feet
      assertTrue(copList.get(0).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0), EPSILON));
      for (int i = 1; i < 4; i++)
         assertTrue(copList.get(0).get(i).containsNaN());
      assertTrue(copList.get(1).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, -0.205), EPSILON));
      assertTrue(copList.get(1).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.06,-0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(1).get(i).containsNaN());
      assertTrue(copList.get(2).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.26, 0.205), EPSILON));
      assertTrue(copList.get(2).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.36, 0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(2).get(i).containsNaN());
      assertTrue(copList.get(3).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.6, -0.205), EPSILON));
      assertTrue(copList.get(3).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.66, -0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(3).get(i).containsNaN());
      testCoPGenerator.clear();
      assertTrue("Planned footsteps not removed", testCoPGenerator.getNumberOfFootstepsRegistered() == 0);
   }

   @Test
   public void testDoubleSupportFootstepPlanMoving()
   {
      sendFootStepMessages(10);
      testCoPGenerator.setInitialCoPPosition(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.1));
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(false, RobotSide.RIGHT);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();
      assertTrue(copList.get(0).get(0).containsNaN());
      assertTrue(copList.get(0).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(0).get(i).containsNaN());
      assertTrue(copList.get(1).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, -0.205), EPSILON));
      assertTrue(copList.get(1).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.06, -0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(1).get(i).containsNaN());
      assertTrue(copList.get(2).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.26, 0.205), EPSILON));
      assertTrue(copList.get(2).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.36, 0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(2).get(i).containsNaN());
      assertTrue(copList.get(3).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.6, -0.205), EPSILON));
      assertTrue(copList.get(3).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.66, -0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(3).get(i).containsNaN());
   }

   @Test
   public void testSingleSupportFootstepPlan()
   {
      int numberOfFootsteps = 10;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepsRegistered() == numberOfFootsteps);
      FramePoint2d initialCoPPosition = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.2);
      testCoPGenerator.setInitialCoPPosition(initialCoPPosition);
      testCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(RobotSide.RIGHT);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();
      assertTrue(copList.get(0).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, -0.205), EPSILON));
      assertTrue(copList.get(0).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.060, -0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(0).get(i).containsNaN());
      assertTrue(copList.get(1).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.26, 0.205), EPSILON));
      assertTrue(copList.get(1).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.36, 0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(1).get(i).containsNaN());
      assertTrue(copList.get(2).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.6, -0.205), EPSILON));
      assertTrue(copList.get(2).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.66, -0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(2).get(i).containsNaN());
      assertTrue(copList.get(3).get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.9, 0.205), EPSILON));
      assertTrue(copList.get(3).get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.96, 0.180), EPSILON));
      for (int i = 2; i < 4; i++)
         assertTrue(copList.get(3).get(i).containsNaN());
   }
}
