package us.ihmc.valkyrieRosControl;

import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieRosControlLowLevelOutputWriter implements JointDesiredOutputWriter
{

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics lowLevelDataHolder)
   {
      
   }

   @Override
   public void initialize()
   {
      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public void writeBefore(long timestamp)
   {
      
   }

   @Override
   public void writeAfter()
   {
      
   }

}
