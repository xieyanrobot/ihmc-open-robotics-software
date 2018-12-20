package us.ihmc.robotics.statistics;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.statistics.OnePassMeanAndStandardDeviation.InsufficientMeasurementsException;

public class OnePassMeanAndStandardDeviationTest
{
   private OnePassMeanAndStandardDeviation meanAndStandardDeviation;

   @BeforeEach
   public void setUp() throws Exception
   {
      meanAndStandardDeviation = new OnePassMeanAndStandardDeviation();
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

	@Test// timeout = 30000
   public void testCalculateMeanAndStandardDeviationExample()
   {
      double[] data =
      {
         65.0, 66.0, 67.0, 69.0, 70.0, 70.0, 70.0, 71.0, 71.0, 72.0, 73.0, 74.0, 76.0
      };

      for (double measurement : data)
      {
         meanAndStandardDeviation.compute(measurement);
      }

      double delta = 1e-10;
      assertEquals(70.3076923076923, meanAndStandardDeviation.getAverage(), delta);
      assertEquals(8.982248520710021, meanAndStandardDeviation.getVariance(), delta);
      assertEquals(2.997039959812018, meanAndStandardDeviation.getStandardDeviation(), delta);
      assertEquals(9.730769230769189, meanAndStandardDeviation.getSampleVariance(), delta);
   }

	@Test// timeout = 30000
   public void testCalculateMeanAndStandardDeviationSingleValue()
   {
      meanAndStandardDeviation.compute(65.0);

      double delta = 1e-10;
      assertEquals(65.0, meanAndStandardDeviation.getAverage(), delta);
      assertEquals(0.0, meanAndStandardDeviation.getVariance(), delta);
      assertEquals(0.0, meanAndStandardDeviation.getStandardDeviation(), delta);
   }

	@Test// timeout = 30000,expected = InsufficientMeasurementsException.class
   public void testCalculateSampleVarianceSingleValue()
   {
      meanAndStandardDeviation.compute(65.0);
      double delta = 1e-10;
      assertEquals(0.0, meanAndStandardDeviation.getSampleVariance(), delta);
   }

	@Test// timeout = 30000,expected = InsufficientMeasurementsException.class
   public void testCalculateAverageNoValue()
   {
      double delta = 1e-10;
      assertEquals(0.0, meanAndStandardDeviation.getAverage(), delta);
   }

	@Test// timeout = 30000,expected = InsufficientMeasurementsException.class
   public void testCalculateVarianceNoValue()
   {
      double delta = 1e-10;
      assertEquals(0.0, meanAndStandardDeviation.getVariance(), delta);
   }

}
