package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SLAMBasics implements SLAMInterface
{
   private final AtomicReference<SLAMFrame> latestSlamFrame = new AtomicReference<>(null);
   protected final NormalOcTree octree;
   private final List<RigidBodyTransformReadOnly> sensorPoses = new ArrayList<>();

   protected PlanarRegionsList planarRegionsMap;
   protected final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   protected final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   protected final PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();

   public SLAMBasics(double octreeResolution)
   {
      octree = new NormalOcTree(octreeResolution);

      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.03);
      planarRegionSegmentationParameters.setMinRegionSize(150);
   }

   public void setNormalEstimationParameters(NormalEstimationParameters normalEstimationParameters)
   {
      octree.setNormalEstimationParameters(normalEstimationParameters);
   }

   public void setPlanarRegionSegmentationParameters(PlanarRegionSegmentationParameters planarRegionSegmentationParameters)
   {
      this.planarRegionSegmentationParameters.set(planarRegionSegmentationParameters);
   }

   public void setPolygonizerParameters(PolygonizerParameters polygonizerParameters)
   {
      this.polygonizerParameters.set(polygonizerParameters);
   }

   public void setConcaveHullFactoryParameters(ConcaveHullFactoryParameters concaveHullFactoryParameters)
   {
      this.concaveHullFactoryParameters.set(concaveHullFactoryParameters);
   }

   @Override
   public void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      SLAMFrame frame = new SLAMFrame(pointCloudMessage);
      latestSlamFrame.set(frame);

      sensorPoses.add(frame.getSensorPose());
   }

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage)
   {
      SLAMFrame frame = new SLAMFrame(getLatestFrame(), pointCloudMessage);

      RigidBodyTransformReadOnly optimizedMultiplier = computeFrameCorrectionTransformer(frame);

      if (optimizedMultiplier == null)
      {
         return false;
      }
      else
      {
         frame.updateOptimizedCorrection(optimizedMultiplier);

         latestSlamFrame.set(frame);

         sensorPoses.add(frame.getSensorPose());

         return true;
      }
   }

   @Override
   public void clear()
   {
      latestSlamFrame.set(null);
      sensorPoses.clear();
      octree.clear();
   }

   public boolean isEmpty()
   {
      if (latestSlamFrame.get() == null)
         return true;
      else
         return false;
   }

   public List<RigidBodyTransformReadOnly> getSensorPoses()
   {
      return sensorPoses;
   }

   public PlanarRegionsList getPlanarRegionsMap()
   {
      return planarRegionsMap;
   }

   public SLAMFrame getLatestFrame()
   {
      return latestSlamFrame.get();
   }

   public double getOctreeResolution()
   {
      return octree.getResolution();
   }

   public NormalOcTree getOctree()
   {
      return octree;
   }
}
