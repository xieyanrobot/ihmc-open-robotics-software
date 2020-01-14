package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.ui.mapping.IhmcSLAMTools;
import us.ihmc.humanoidBehaviors.ui.mapping.SimulatedStereoVisionPointCloudMessageLibrary;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAMFrame;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.randomICP.RICPSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.IhmcSLAMViewer;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;

public class RandomICPSLAMUnitTest
{
   @Test
   public void testComputeOverlappedArea()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      IhmcSLAMFrame previousFrame = new IhmcSLAMFrame(messages.get(49));
      IhmcSLAMFrame frame = new IhmcSLAMFrame(previousFrame, messages.get(50));

      List<Point3D> pointsOutOfPreviousWindow = new ArrayList<>();
      List<Point3D> pointsInPreviousWindow = new ArrayList<>();

      double windowWidth = 1.0;
      double windowHeight = 0.4;
      double windowDepth = 0.5;
      ConvexPolygon2D previousWindow = new ConvexPolygon2D();
      previousWindow.addVertex(windowWidth / 2, windowHeight / 2);
      previousWindow.addVertex(windowWidth / 2, -windowHeight / 2);
      previousWindow.addVertex(-windowWidth / 2, -windowHeight / 2);
      previousWindow.addVertex(-windowWidth / 2, windowHeight / 2);
      previousWindow.update();

      RigidBodyTransformReadOnly previousSensorPoseToWorld = previousFrame.getInitialSensorPoseToWorld();
      Point3D[] convertedPointsToPreviousSensorPose = IhmcSLAMTools.createConvertedPointsToSensorPose(previousSensorPoseToWorld, frame.getOriginalPointCloud());

      int numberOfPointsInPreviousView = 0;
      for (int i = 0; i < convertedPointsToPreviousSensorPose.length; i++)
      {
         Point3D pointInPreviousView = convertedPointsToPreviousSensorPose[i];
         if (previousWindow.isPointInside(pointInPreviousView.getX(), pointInPreviousView.getY()) && pointInPreviousView.getZ() > windowDepth)
         {
            previousSensorPoseToWorld.transform(pointInPreviousView);
            pointsInPreviousWindow.add(new Point3D(pointInPreviousView));
            numberOfPointsInPreviousView++;
         }
         else
         {
            previousSensorPoseToWorld.transform(pointInPreviousView);
            pointsOutOfPreviousWindow.add(new Point3D(pointInPreviousView));
         }
      }
      System.out.println(convertedPointsToPreviousSensorPose.length + " " + numberOfPointsInPreviousView);
      Point3D[] inliers = new Point3D[pointsInPreviousWindow.size()];
      Point3D[] outliers = new Point3D[pointsOutOfPreviousWindow.size()];
      for (int i = 0; i < inliers.length; i++)
      {
         inliers[i] = new Point3D(pointsInPreviousWindow.get(i));
      }
      for (int i = 0; i < outliers.length; i++)
      {
         outliers[i] = new Point3D(pointsOutOfPreviousWindow.get(i));
      }

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      slamViewer.addSensorPose(previousFrame.getSensorPose(), Color.BLUE);
      slamViewer.addSensorPose(frame.getSensorPose(), Color.GREEN);
      slamViewer.addPointCloud(previousFrame.getPointCloud(), Color.BLUE);
      slamViewer.addPointCloud(frame.getPointCloud(), Color.GREEN);
      
      slamViewer.addPointCloud(inliers, Color.RED);
      slamViewer.addPointCloud(outliers, Color.BLACK);
      
      slamViewer.start("testComputeOverlappedArea");
      ThreadTools.sleepForever();
   }

   @Test
   public void testOptimizationForSimulatedPointCloud()
   {
      double movingForward = 0.1;
      double fixedHeight = 1.0;

      double sensorPitchAngle = Math.toRadians(90.0 + 70.0);
      double stairHeight = 0.3;
      double stairWidth = 0.5;
      double stairLength = 0.25;

      RigidBodyTransform sensorPoseOne = new RigidBodyTransform();
      sensorPoseOne.setTranslation(0.0, 0.0, fixedHeight);
      sensorPoseOne.appendPitchRotation(sensorPitchAngle);
      StereoVisionPointCloudMessage messageOne = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseOne,
                                                                                                                          new RigidBodyTransform(), stairHeight,
                                                                                                                          stairWidth, stairLength, stairLength,
                                                                                                                          true);

      double translationX = movingForward / 2;
      double translationY = 0.0;
      double translationZ = 0.0;
      double rotateY = Math.toRadians(0.0);
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      preMultiplier.setTranslation(translationX, translationY, translationZ);
      preMultiplier.appendPitchRotation(rotateY);

      RigidBodyTransform sensorPoseTwo = new RigidBodyTransform();
      sensorPoseTwo.setTranslation(movingForward, 0.0, fixedHeight);
      sensorPoseTwo.appendPitchRotation(sensorPitchAngle);

      RigidBodyTransform randomTransformer = createRandomDriftedTransform(new Random(0612L), 0.1, 10.0);
      preMultiplier.multiply(randomTransformer);
      sensorPoseTwo.multiply(randomTransformer);

      StereoVisionPointCloudMessage driftedMessageTwo = SimulatedStereoVisionPointCloudMessageLibrary.generateMessageSimpleStair(sensorPoseTwo, preMultiplier,
                                                                                                                                 stairHeight, stairWidth,
                                                                                                                                 stairLength - movingForward,
                                                                                                                                 stairLength + movingForward,
                                                                                                                                 true);

      double octreeResolution = 0.01;
      RICPSLAM slam = new RICPSLAM(octreeResolution);

      slam.addFirstFrame(messageOne);
      slam.addFrame(driftedMessageTwo);

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();
      slamViewer.addPointCloud(slam.getSLAMFrame(0).getOriginalPointCloud(), Color.BLUE);
      slamViewer.addPointCloud(slam.getSLAMFrame(1).getOriginalPointCloud(), Color.BLACK);
      slamViewer.addSensorPose(slam.getSLAMFrame(1).getOriginalSensorPose(), Color.BLACK);

      slamViewer.addPointCloud(slam.getSLAMFrame(1).getPointCloud(), Color.GREEN);
      slamViewer.addSensorPose(slam.getSLAMFrame(1).getSensorPose(), Color.GREEN);

      Point3D[] ruler = new Point3D[50];
      for (int i = 0; i < 50; i++)
         ruler[i] = new Point3D(0.15, 0.0, (double) i * octreeResolution);
      slamViewer.addPointCloud(ruler, Color.RED);

      slamViewer.start("testOptimizationForSimulatedPointCloud");

      ThreadTools.sleepForever();
   }

   @Test
   public void testOptimizationForRealData()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.01;
      RICPSLAM slam = new RICPSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(49));
      slam.addFrame(messages.get(50));

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      slamViewer.addSensorPose(slam.getSensorPoses().get(0), Color.BLUE);
      slamViewer.addSensorPose(slam.getSensorPoses().get(1), Color.GREEN);

      slamViewer.addPointCloud(slam.getPointCloudMap().get(0), Color.BLUE);
      slamViewer.addPointCloud(slam.getOriginalPointCloudMap().get(1), Color.BLACK);
      slamViewer.addPointCloud(slam.getPointCloudMap().get(1), Color.GREEN);

      Point3D[] sourcePoints = new Point3D[slam.sourcePointsToWorld.size()];
      for (int i = 0; i < sourcePoints.length; i++)
         sourcePoints[i] = new Point3D(slam.sourcePointsToWorld.get(i));
      slamViewer.addPointCloud(sourcePoints, Color.RED);

      slamViewer.start("testOptimizationForRealData");
      ThreadTools.sleepForever();
   }

   @Test
   public void testRandomICPSLAMEndToEnd()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      IhmcSLAM slam = new RICPSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(0));
      for (int i = 1; i < 60; i++)
      {
         System.out.println(" ## add frame " + i);
         slam.addFrame(messages.get(i));
      }

      slam.updatePlanarRegionsMap();

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         slamViewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
         slamViewer.addSensorPose(slam.getSensorPoses().get(i), Color.BLUE);
      }
      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());

      slamViewer.start("testRandomICPSLAMEndToEnd");
      ThreadTools.sleepForever();
   }

   private RigidBodyTransform createRandomDriftedTransform(Random random, double positionBound, double angleBoundDegree)
   {
      RigidBodyTransform randomTransform = new RigidBodyTransform();
      int positionAxis = random.nextInt(2);
      int angleAxis = random.nextInt(2);

      double randomAngle = 2 * Math.toRadians(angleBoundDegree) * (random.nextDouble() - 0.5);
      if (angleAxis == 0)
         randomTransform.appendRollRotation(randomAngle);
      if (angleAxis == 1)
         randomTransform.appendPitchRotation(randomAngle);
      else
         randomTransform.appendYawRotation(randomAngle);

      double randomTranslation = 2 * positionBound * (random.nextDouble() - 0.5);
      if (positionAxis == 0)
         randomTransform.appendTranslation(randomTranslation, 0.0, 0.0);
      if (positionAxis == 1)
         randomTransform.appendTranslation(0.0, randomTranslation, 0.0);
      else
         randomTransform.appendTranslation(0.0, 0.0, randomTranslation);

      System.out.println("positionAxis " + positionAxis + " " + randomTranslation);
      System.out.println("angleAxis " + angleAxis + " " + Math.toDegrees(randomAngle));

      return randomTransform;
   }
}
