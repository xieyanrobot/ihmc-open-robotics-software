package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dijkstra.DijkstraVisibilityGraphPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface VisibilityGraphsParameters
{
   double getMaxInterRegionConnectionLength();

   double getNormalZThresholdForAccessibleRegions();

   //TODO: This parameter doesn't seem to do anything. It seems this one is used: getNavigableExtrusionDistanceCalculator 
   double getExtrusionDistance();

   double getExtrusionDistanceIfNotTooHighToStep();

   double getTooHighToStepDistance();

   double getClusterResolution();

   default double getExplorationDistanceFromStartGoal()
   {
      return Double.POSITIVE_INFINITY;
   }

   default double getPlanarRegionMinArea()
   {
      return 0.0;
   }

   default int getPlanarRegionMinSize()
   {
      return 0;
   }

   /**
    * Defines the angle from which two regions are considered orthogonal.
    * <p>
    * It is used to determine if a region should be projected onto another as a polygon or a line.
    * </p>
    * <p>
    * It should be close to 90 degrees.
    * </p>
    * 
    * @return the angle threshold to use to determine if a line or polygon projection method should
    *         be used.
    */
   default double getRegionOrthogonalAngle()
   {
      return Math.toRadians(75.0);
   }

   /**
    * This epsilon is is used when searching to which region the start/goal belongs to.
    * <p>
    * A positive value corresponds to growing all the regions before testing if the start/goal is
    * inside.
    * </p>
    * 
    * @return the value of the epsilon to use.
    */
   default double getSearchHostRegionEpsilon()
   {
      return 0.03;
   }

   /**
    * The constant extrusion distance to use when extruding the hull of a navigable region.
    * 
    * @return
    */
   default NavigableExtrusionDistanceCalculator getNavigableExtrusionDistanceCalculator()
   {
      return new NavigableExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(PlanarRegion navigableRegionToBeExtruded)
         {
            return getExtrusionDistanceIfNotTooHighToStep();
         }
      };
   }

   /**
    * This calculator is used when extruding the projection of an obstacle onto a navigable region.
    * 
    * @return the calculator use for obstacle extrusion.
    */
   default ObstacleExtrusionDistanceCalculator getObstacleExtrusionDistanceCalculator()
   {
      return (pointToExtrude, obstacleHeight) -> {
         if (obstacleHeight < 0.0)
         {
            return 0.0;
         }
         else if (obstacleHeight < getTooHighToStepDistance())
         {
            return getExtrusionDistanceIfNotTooHighToStep();
         }
         else
         {
            return getExtrusionDistance();
         }
      };
   }

   default NavigableRegionFilter getNavigableRegionFilter()
   {
      return new NavigableRegionFilter()
      {
         @Override
         public boolean isPlanarRegionNavigable(PlanarRegion query, List<PlanarRegion> allOtherRegions)
         {
            return query.getNormal().getZ() >= getNormalZThresholdForAccessibleRegions();
         }
      };
   }

   default InterRegionConnectionFilter getInterRegionConnectionFilter()
   {
      return new InterRegionConnectionFilter()
      {
         private final double maxLength = getMaxInterRegionConnectionLength();
         private final double maxLengthSquared = MathTools.square(maxLength);
         private final double maxDeltaHeight = getTooHighToStepDistance();

         @Override
         public boolean isConnectionValid(ConnectionPoint3D source, ConnectionPoint3D target)
         {
            if (Math.abs(source.getZ() - target.getZ()) > maxDeltaHeight)
               return false;
            if (source.distanceSquared(target) > maxLengthSquared)
               return false;

            return true;
         }

         @Override
         public double getMaximumInterRegionConnetionDistance()
         {
            return maxLength;
         }
      };
   }

   default PlanarRegionFilter getPlanarRegionFilter()
   {
      return new PlanarRegionFilter()
      {
         @Override
         public boolean isPlanarRegionRelevant(PlanarRegion region)
         {
            if (region.getConcaveHullSize() < getPlanarRegionMinSize())
               return false;
            if (!Double.isFinite(getPlanarRegionMinArea()) || getPlanarRegionMinArea() <= 0.0)
               return true;
            return PlanarRegionTools.computePlanarRegionArea(region) >= getPlanarRegionMinArea();
         }
      };
   }

   default ObstacleRegionFilter getObstacleRegionFilter()
   {
      return new ObstacleRegionFilter()
      {
         double canDuckUnderHeight = 2.0;
         double canEasilyStepOverHeight = 0.03;

         @Override
         public boolean isRegionValidObstacle(PlanarRegion query, PlanarRegion navigableRegion)
         {
            //TOOD: ++++++JEP: Lots of bugs here. Need to clean up ConvexPolygon stuff to find distances and if overlapping more nicely...
            //TODO: Get rid of these magic numbers and make them parameters somewhere. Make sure the overlapping region check is larger than getMaxInterRegionConnectionLength() 
            //TODO: BodyPathPlannerEnvironment crash when the number is set to 1.0. But should work fine all the same...
            //TOOD: This check should just be an approximation and should be ok for false positives. In fact, just returning true should be ok. Check that.
            //TODO: But somehow that's not right, since if we chang 0.25 to 1.0 below, we get a Runtime Exception: Tried to create a line from two coincidal points!?
            if (!PlanarRegionTools.isRegionAOverlapingWithRegionB(query, navigableRegion, 0.25)) //1.0))
               return false;

            double minimumHeight = PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(query, navigableRegion);
            if (minimumHeight > canDuckUnderHeight )
               return false;
            
            if (!PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(query, navigableRegion, canEasilyStepOverHeight))
               return false;

            return true;
         }
      };
   }

   default VisibilityGraphPathPlanner getPathPlanner()
   {
      return new DijkstraVisibilityGraphPlanner();
      //      return JGraphTools.getJGraphPlanner();
   }
}
