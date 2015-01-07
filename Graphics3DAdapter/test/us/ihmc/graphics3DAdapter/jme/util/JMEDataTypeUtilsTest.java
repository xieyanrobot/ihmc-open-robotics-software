package us.ihmc.graphics3DAdapter.jme.util;

import java.util.Random;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.test.JUnitTools;

import com.jme3.math.Transform;

public class JMEDataTypeUtilsTest
{
   @Test(timeout=300000)
   public void testTransforms()
   {
      for (int i = 0; i < 1000; i++)
      {
         RigidBodyTransform randomTransform = RandomTools.generateRandomTransform(new Random(-2346283641976L));
         Transform jmeVersion = JMEDataTypeUtils.j3dTransform3DToJMETransform(randomTransform);
         RigidBodyTransform resultTransform = JMEDataTypeUtils.jmeTransformToTransform3D(jmeVersion);
         JUnitTools.assertTransformEquals(randomTransform, resultTransform, 1e-6);
      }
   }

}
