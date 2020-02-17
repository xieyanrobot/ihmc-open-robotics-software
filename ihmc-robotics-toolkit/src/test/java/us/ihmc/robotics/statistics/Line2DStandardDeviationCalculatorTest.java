package us.ihmc.robotics.statistics;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLine2D;

import java.util.Random;
import java.util.Vector;

import static us.ihmc.robotics.Assert.assertEquals;

public class Line2DStandardDeviationCalculatorTest
{
   private static final double epsilon = 1e-6;

   @Test
   public void testNoVariance()
   {
      YoVariableRegistry testRegistry = new YoVariableRegistry(getClass().getSimpleName());
      YoFrameLine2D valueProvider = new YoFrameLine2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Line2DStandardDeviationCalculator calculator = new Line2DStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Vector2D direction = new Vector2D(17.3, 5.6);
      Point2D position = new Point2D(17.3, 5.6);
      direction.normalize();
      valueProvider.setDirection(direction);
      valueProvider.setPoint(position);

      for (int i = 0; i < numberOfValues; i++)
      {
         calculator.update();
      }

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(valueProvider, calculator.getLineMean(), epsilon);
      assertEquals(0.0, calculator.getDirectionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getPositionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getDirectionVariance(), epsilon);
      assertEquals(0.0, calculator.getPositionVariance(), epsilon);
   }

   @Test
   public void testNoVarianceWithOppositeDirections()
   {
      YoVariableRegistry testRegistry = new YoVariableRegistry(getClass().getSimpleName());
      YoFrameLine2D valueProvider = new YoFrameLine2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Line2DStandardDeviationCalculator calculator = new Line2DStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Vector2D direction = new Vector2D(17.3, 5.6);
      Point2D position = new Point2D(17.3, 5.6);
      direction.normalize();
      valueProvider.setDirection(direction);
      valueProvider.setPoint(position);

      for (int i = 0; i < numberOfValues; i++)
      {
         direction.negate();
         valueProvider.setDirection(direction);
         calculator.update();
      }

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(valueProvider, calculator.getLineMean(), epsilon);
      assertEquals(0.0, calculator.getDirectionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getPositionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getDirectionVariance(), epsilon);
      assertEquals(0.0, calculator.getPositionVariance(), epsilon);
   }

   @Test
   public void testNoVarianceAlongLine()
   {
      YoVariableRegistry testRegistry = new YoVariableRegistry(getClass().getSimpleName());
      YoFrameLine2D valueProvider = new YoFrameLine2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Line2DStandardDeviationCalculator calculator = new Line2DStandardDeviationCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Vector2D direction = new Vector2D(17.3, 5.6);
      Point2D position = new Point2D(17.3, 5.6);
      direction.normalize();
      valueProvider.setDirection(direction);
      valueProvider.setPoint(position);

      Line2D originalValue = new Line2D(valueProvider);

      Random random = new Random(1738L);

      for (int i = 0; i < numberOfValues; i++)
      {
         Point2D modifiedPosition = new Point2D(position);
         Vector2D positionModification = new Vector2D(direction);
         position.scale(RandomNumbers.nextDouble(random, 10.0));
         modifiedPosition.add(positionModification);

         valueProvider.setPoint(modifiedPosition);

         calculator.update();
      }

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(originalValue, calculator.getLineMean(), epsilon);
      assertEquals(0.0, calculator.getDirectionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getPositionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getDirectionVariance(), epsilon);
      assertEquals(0.0, calculator.getPositionVariance(), epsilon);
   }
}
