package us.ihmc.footstepPlanning.log.graphics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;

public class BoxGraphic
{
   private final YoFramePose3D boxPose;
   private final YoDouble sizeX, sizeY, sizeZ;

   public BoxGraphic(String namePrefix, YoVariableRegistry registry)
   {
      this.boxPose = new YoFramePose3D(namePrefix + "_Box", ReferenceFrame.getWorldFrame(), registry);
      this.sizeX = new YoDouble(namePrefix + "SizeX", registry);
      this.sizeY = new YoDouble(namePrefix + "SizeY", registry);
      this.sizeZ = new YoDouble(namePrefix + "SizeZ", registry);
   }

   public YoFramePose3D getBoxPose()
   {
      return boxPose;
   }

   public YoDouble getSizeX()
   {
      return sizeX;
   }

   public YoDouble getSizeY()
   {
      return sizeY;
   }

   public YoDouble getSizeZ()
   {
      return sizeZ;
   }
}
