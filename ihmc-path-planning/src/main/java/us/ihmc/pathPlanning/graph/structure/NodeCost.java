package us.ihmc.pathPlanning.graph.structure;

public class NodeCost
{
   private final double cost;

   public NodeCost(double cost)
   {
      this.cost = cost;
   }

   public double getNodeCost()
   {
      return cost;
   }
}
