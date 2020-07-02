package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import javafx.scene.paint.Color;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.graphics.SphereGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;
import java.util.function.UnaryOperator;

public class FootstepNodeChecker
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapAndWiggler snapper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final PlanarRegionCliffAvoider cliffAvoider;
   private final ObstacleBetweenNodesChecker obstacleBetweenNodesChecker;
   private final FootstepNodeBodyCollisionDetector collisionDetector;
   private final GoodFootstepPositionChecker goodPositionChecker;

   private PlanarRegionsList planarRegionsList = null;

   private final FootstepNodeSnapData candidateNodeSnapData = FootstepNodeSnapData.identityData();
   private final YoEnum<BipedalFootstepPlannerNodeRejectionReason> rejectionReason = new YoEnum<>("rejectionReason", "", registry, BipedalFootstepPlannerNodeRejectionReason.class, true);
   private final YoDouble footAreaPercentage = new YoDouble("footAreaPercentage", registry);
   private final YoInteger footstepIndex = new YoInteger("footstepIndex", registry);
   private final SphereGraphic sphereGraphic = new SphereGraphic("testSphere", Color.color(0.5, .5, 0.5, 0.5), registry);

   public FootstepNodeChecker(FootstepPlannerParametersReadOnly parameters,
                              SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapAndWiggler snapper, YoVariableRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.footPolygons = footPolygons;
      this.cliffAvoider = new PlanarRegionCliffAvoider(parameters, snapper, footPolygons);
      this.obstacleBetweenNodesChecker = new ObstacleBetweenNodesChecker(parameters, snapper);
      this.collisionDetector = new FootstepNodeBodyCollisionDetector(parameters);
      this.goodPositionChecker = new GoodFootstepPositionChecker(parameters, snapper, registry);
      parentRegistry.addChild(registry);
   }

   public boolean isNodeValid(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      if (stanceNode != null && candidateNode.getRobotSide() == stanceNode.getRobotSide())
      {
         throw new RuntimeException(getClass().getSimpleName() + " received a pair stance and swing footsteps both on the " + candidateNode.getRobotSide() + " side");
      }

      clearLoggedVariables();
      doValidityCheck(candidateNode, stanceNode);
      sphereGraphic.getCenter().set(candidateNode.getX(), candidateNode.getY(), 0.2);
      sphereGraphic.getRadius().set(0.1);

      return rejectionReason.getValue() == null;
   }

   public void onIterationStart(FootstepNode footstepNode)
   {
      footstepIndex.set(footstepNode.getChildNodes().size() - 1);
   }

   private void doValidityCheck(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(candidateNode, stanceNode, parameters.getWiggleWhilePlanning());
      candidateNodeSnapData.set(snapData);

      if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return;
      }

      // Check valid snap
      if (candidateNodeSnapData.getSnapTransform().containsNaN())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return;
      }

      // Check incline
      RigidBodyTransform snappedSoleTransform = candidateNodeSnapData.getSnappedNodeTransform(candidateNode);
      double minimumSurfaceNormalZ = Math.cos(parameters.getMinimumSurfaceInclineRadians());
      if (snappedSoleTransform.getM22() < minimumSurfaceNormalZ)
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);
         return;
      }

      // Check snap area
      ConvexPolygon2D footholdAfterSnap = candidateNodeSnapData.getCroppedFoothold();
      double croppedFootArea = footholdAfterSnap.getArea();
      double fullFootArea = footPolygons.get(candidateNode.getRobotSide()).getArea();
      footAreaPercentage.set(croppedFootArea / fullFootArea);

      double epsilonAreaPercentage = 1e-4;
      if (!footholdAfterSnap.isEmpty() && footAreaPercentage.getValue() < (parameters.getMinimumFootholdPercent() - epsilonAreaPercentage))
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
         return;
      }

      // Check for ankle collision
      cliffAvoider.setPlanarRegionsList(planarRegionsList);
      if(!cliffAvoider.isNodeValid(candidateNode))
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM);
         return;
      }

      if(stanceNode == null)
      {
         return;
      }

      footstepIndex.set(stanceNode.getChildNodes().size());

      // Check snapped footstep placement
      if (!goodPositionChecker.isNodeValid(candidateNode, stanceNode))
      {
         rejectionReason.set(goodPositionChecker.getRejectionReason());
         return;
      }

      // Check for obstacle collisions (vertically extruded line between steps)
      if (parameters.checkForPathCollisions())
      {
         obstacleBetweenNodesChecker.setPlanarRegions(planarRegionsList);
         try
         {
            if (!obstacleBetweenNodesChecker.isNodeValid(candidateNode, stanceNode))
            {
               rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_BODY);
               return;
            }
         }
         catch(Exception e)
         {
            e.printStackTrace();
         }
      }

      // Check for bounding box collisions
      if (parameters.checkForBodyBoxCollisions())
      {
         if (boundingBoxCollisionDetected(candidateNode, stanceNode))
         {
            rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY);
            return;
         }
     }
   }

   private boolean boundingBoxCollisionDetected(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      FootstepNodeSnapData stanceNodeSnapData = snapper.snapFootstepNode(stanceNode, null, parameters.getWiggleWhilePlanning());
      if (stanceNodeSnapData == null)
      {
         return false;
      }

      double candidateNodeHeight = FootstepNodeTools.getSnappedNodeHeight(candidateNode, candidateNodeSnapData.getSnapTransform());
      double stanceNodeHeight = FootstepNodeTools.getSnappedNodeHeight(stanceNode, stanceNodeSnapData.getSnapTransform());
      List<BodyCollisionData> collisionData = collisionDetector.checkForCollision(candidateNode,
                                                                                  stanceNode,
                                                                                  candidateNodeHeight,
                                                                                  stanceNodeHeight,
                                                                                  parameters.getNumberOfBoundingBoxChecks());
      for (int i = 0; i < collisionData.size(); i++)
      {
         if (collisionData.get(i).isCollisionDetected())
         {
            return true;
         }
      }

      return false;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      collisionDetector.setPlanarRegionsList(planarRegionsList);
   }

   public void setParentNodeSupplier(UnaryOperator<FootstepNode> parentNodeSupplier)
   {
      this.goodPositionChecker.setParentNodeSupplier(parentNodeSupplier);
   }

   private void clearLoggedVariables()
   {
      footAreaPercentage.setToNaN();
      rejectionReason.set(null);
      footstepIndex.set(-1);

      candidateNodeSnapData.clear();
      goodPositionChecker.clearLoggedVariables();
   }
}
