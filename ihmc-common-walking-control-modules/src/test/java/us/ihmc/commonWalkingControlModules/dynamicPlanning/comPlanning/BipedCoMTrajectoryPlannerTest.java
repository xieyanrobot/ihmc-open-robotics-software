package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;

public class BipedCoMTrajectoryPlannerTest
{
   @Test
   public void testFancySteps()
   {
      new BipedCoMTrajectoryPlannerVisualizer(BipedCoMTrajectoryPlannerVisualizer::createFancySteps);
   }

   @Test
   public void testRunningSteps()
   {
      new BipedCoMTrajectoryPlannerVisualizer(BipedCoMTrajectoryPlannerVisualizer::createSteps);
   }
}
