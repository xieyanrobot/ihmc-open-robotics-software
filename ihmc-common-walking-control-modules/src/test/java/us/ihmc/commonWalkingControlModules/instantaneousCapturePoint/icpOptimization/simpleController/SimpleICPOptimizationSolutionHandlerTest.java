package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;

import static org.junit.Assert.assertTrue;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SimpleICPOptimizationSolutionHandlerTest
{
   private YoVariableRegistry registry = new YoVariableRegistry("robert");

   private ICPOptimizationParameters parameters;
   private SimpleICPOptimizationSolutionHandler solutionHandler;
   private SimpleICPOptimizationQPSolver solver;

   private void setupTest(double deadbandSize)
   {
      parameters = new TestICPOptimizationParameters(deadbandSize);
      solutionHandler = new SimpleICPOptimizationSolutionHandler(parameters, false, "test", registry);
      solver = new SimpleICPOptimizationQPSolver(parameters, 4, false);
   }

   private ArrayList<Footstep> createFootsteps(double length, double width, int numberOfSteps)
   {
      ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
      RobotSide robotSide = RobotSide.LEFT;
      for (int i = 0; i < numberOfSteps; i++)
      {
         FramePose footstepPose = new FramePose(ReferenceFrame.getWorldFrame());

         footstepPose.setPosition(length * (i + 1), robotSide.negateIfRightSide(0.5 * width), 0.0);
         upcomingFootsteps.add(new Footstep(robotSide, footstepPose, false));

         FramePoint2D referenceFootstepPosition = new FramePoint2D();
         footstepPose.getPosition2dIncludingFrame(referenceFootstepPosition);

         robotSide = robotSide.getOppositeSide();
      }

      return upcomingFootsteps;
   }

   private ArrayList<FramePoint2D> createReferenceLocations(ArrayList<Footstep> upcomingFootsteps)
   {
      ArrayList<FramePoint2D> referenceFootstepLocations = new ArrayList<>();

      for (int i = 0; i < upcomingFootsteps.size(); i++)
      {
         FramePoint2D referenceLocation = new FramePoint2D();
         upcomingFootsteps.get(i).getPosition2d(referenceLocation);
         referenceFootstepLocations.add(referenceLocation);
      }

      return referenceFootstepLocations;
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testWellWithinDeadband()
   {
      double scale = 0.2;
      double deadbandSize = 0.05;
      runTest(scale, deadbandSize);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testALittleWithinDeadband()
   {
      double scale = 0.9;
      double deadbandSize = 0.05;
      runTest(scale, deadbandSize);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testJustWithinDeadband()
   {
      double scale = 0.99;
      double deadbandSize = 0.05;
      runTest(scale, deadbandSize);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testRightOnDeadband()
   {
      double scale = 1.0;
      double deadbandSize = 0.05;
      runTest(scale, deadbandSize);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testJustOutsideDeadband()
   {
      double scale = 1.01;
      double deadbandSize = 0.05;
      runTest(scale, deadbandSize);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testALittleOutsideDeadband()
   {
      double scale = 1.05;
      double deadbandSize = 0.05;
      runTest(scale, deadbandSize);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testWellOutsideDeadband()
   {
      double scale = 1.5;
      double deadbandSize = 0.05;
      runTest(scale, deadbandSize);
   }

   private void runTest(double scale, double deadbandSize)
   {
      setupTest(deadbandSize);

      double stepLength = 0.5;
      double stanceWidth = 0.2;
      int numberOfSteps = 3;
      ArrayList<YoFramePoint2d> foostepSolutions = new ArrayList<>();
      ArrayList<FramePoint2D> unclippedFootstepSolutions = new ArrayList<>();

      for (int i = 0; i < numberOfSteps; i++)
      {
         foostepSolutions.add(new YoFramePoint2d("footstepSolution" + i, ReferenceFrame.getWorldFrame(), registry));
         unclippedFootstepSolutions.add(new FramePoint2D(ReferenceFrame.getWorldFrame()));
      }

      ArrayList<Footstep> upcomingFootsteps = createFootsteps(stepLength, stanceWidth, numberOfSteps);
      ArrayList<FramePoint2D> referenceFootstepPositions = createReferenceLocations(upcomingFootsteps);

      double recursionMultiplier = Math.exp(-3.0 * 0.5);
      setupFeedbackTask(1000.0, stanceWidth);
      setupFootstepAdjustmentTask(5.0, recursionMultiplier, referenceFootstepPositions.get(0));

      FrameVector2D currentICPError = new FrameVector2D(ReferenceFrame.getWorldFrame(), scale * deadbandSize, 0.0);
      currentICPError.scale(recursionMultiplier);

      FramePoint2D perfectCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), -0.1, 0.0);
      try
      {
         solver.compute(currentICPError, perfectCMP);
      }
      catch (NoConvergenceException e)
      {
      }

      solutionHandler.extractFootstepSolutions(foostepSolutions, unclippedFootstepSolutions, upcomingFootsteps, 1, solver);
      FrameVector2D copFeedback = new FrameVector2D();
      solver.getCoPFeedbackDifference(copFeedback);

      // this should be within the deadband
      FramePoint2D expectedUnclippedSolution = new FramePoint2D(referenceFootstepPositions.get(0));
      expectedUnclippedSolution.add(scale * deadbandSize, 0.0);
      FramePoint2D expectedClippedSolution = new FramePoint2D(referenceFootstepPositions.get(0));
      expectedClippedSolution.add(Math.max(scale - 1.0, 0.0) * deadbandSize, 0.0);

      assertTrue(foostepSolutions.get(0).epsilonEquals(expectedClippedSolution, 1e-3));
      assertTrue(unclippedFootstepSolutions.get(0).epsilonEquals(expectedUnclippedSolution, 1e-3));
   }

   private void setupFeedbackTask(double weight, double stanceWidth)
   {
      double footLength = 0.2;
      double footWidth = 0.1;

      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
      supportPolygon.addVertex(new Point2D(0.5 * footLength, 0.5 * footWidth - 0.5 * stanceWidth));
      supportPolygon.addVertex(new Point2D(0.5 * footLength, -0.5 * footWidth - 0.5 * stanceWidth));
      supportPolygon.addVertex(new Point2D(-0.5 * footLength, 0.5 * footWidth - 0.5 * stanceWidth));
      supportPolygon.addVertex(new Point2D(-0.5 * footLength, -0.5 * footWidth - 0.5 * stanceWidth));
      supportPolygon.update();

      double feedbackGain = 2.5;
      double dynamicsWeight = 10000.0;

      solver.setFeedbackConditions(feedbackGain, weight, dynamicsWeight);
      solver.addSupportPolygon(supportPolygon);
   }

   private void setupFootstepAdjustmentTask(double weight, double recursionMultiplier, FramePoint2D referenceFootstepPosition)
   {
      solver.setFootstepAdjustmentConditions(recursionMultiplier, weight, referenceFootstepPosition);
   }


   private class TestICPOptimizationParameters extends ICPOptimizationParameters
   {
      private final double deadbandSize;

      public TestICPOptimizationParameters(double deadbandSize)
      {
         this.deadbandSize = deadbandSize;
      }

      @Override public boolean useSimpleOptimization()
      {
         return false;
      }

      @Override public int getMaximumNumberOfFootstepsToConsider()
      {
         return 5;
      }

      @Override public int numberOfFootstepsToConsider()
      {
         return 0;
      }

      @Override public double getForwardFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getLateralFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getFootstepRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackForwardWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackLateralWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackParallelGain()
      {
         return 2.0;
      }

      @Override public double getFeedbackOrthogonalGain()
      {
         return 3.0;
      }

      @Override public double getDynamicRelaxationWeight()
      {
         return 1000.0;
      }

      @Override public double getDynamicRelaxationDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override public double getAngularMomentumMinimizationWeight()
      {
         return 0.5;
      }

      @Override public boolean scaleStepRegularizationWeightWithTime()
      {
         return false;
      }

      @Override public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override public boolean scaleUpcomingStepWeights()
      {
         return false;
      }

      @Override public boolean useFeedbackRegularization()
      {
         return false;
      }

      @Override public boolean useStepAdjustment()
      {
         return true;
      }

      @Override public boolean useTimingOptimization()
      {
         return false;
      }

      @Override public boolean useAngularMomentum()
      {
         return false;
      }

      @Override public boolean useFootstepRegularization()
      {
         return false;
      }

      @Override public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }

      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.001;
      }

      @Override public double getAdjustmentDeadband()
      {
         return deadbandSize;
      }
   };
}
