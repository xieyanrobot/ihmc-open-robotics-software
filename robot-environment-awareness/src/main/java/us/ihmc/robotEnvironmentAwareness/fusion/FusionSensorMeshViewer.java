package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.fusion.data.FusedSuperPixelImageBuilder;
import us.ihmc.robotEnvironmentAwareness.fusion.data.RawSuperPixelImageBuilder;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.DetectedObjectViewer;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.LidarScanViewer;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;
import us.ihmc.ros2.Ros2Node;

public class FusionSensorMeshViewer
{
   private static final int SLOW_PACE_UPDATE_PERIOD = 250;
   private static final int MEDIUM_PACE_UPDATE_PERIOD = 100;
   private static final int HIGH_PACE_UPDATE_PERIOD = 10;

   private final REAUIMessager reaMessager;
   
   private final Group root = new Group();

   private final LidarScanViewer lidarScanViewer;
   private final StereoVisionPointCloudViewer stereoVisionPointCloudViewer;
   private final DetectedObjectViewer detectedObjectViewer;
   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;
   private final RawSuperPixelImageBuilder rawPixelImageBuilder;
   private final FusedSuperPixelImageBuilder fusedPixelImageMeshBuilder;

   private final MeshView planarRegionMeshView = new MeshView();
   private final MeshView fusedSuperPixelMeshView = new MeshView();
   private final MeshView rawSuperPixelMeshView = new MeshView();

   private final AnimationTimer renderMeshAnimation;
   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   public FusionSensorMeshViewer(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager, REAUIMessager reaMessager) throws Exception
   {
      this.reaMessager = reaMessager;
      lidarScanViewer = new LidarScanViewer(REAModuleAPI.LidarScanState, reaMessager);
      stereoVisionPointCloudViewer = new StereoVisionPointCloudViewer(REAModuleAPI.StereoVisionPointCloudState, reaMessager);
      detectedObjectViewer = new DetectedObjectViewer(ros2Node);
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(reaMessager);
      rawPixelImageBuilder = new RawSuperPixelImageBuilder(messager);
      fusedPixelImageMeshBuilder = new FusedSuperPixelImageBuilder(messager);

      messager.registerTopicListener(LidarImageFusionAPI.ClearREA, (content) -> clear());

      AtomicReference<Boolean> showPlanarRegions = messager.createInput(LidarImageFusionAPI.ShowPlanarRegions, true);
      AtomicReference<Boolean> showFusedSuperPixelData = messager.createInput(LidarImageFusionAPI.ShowFusedSuperPixelData, false);
      AtomicReference<Boolean> showRawSuperPixelData = messager.createInput(LidarImageFusionAPI.ShowRawSuperPixelData, false);


      Node lidarScanRootNode = lidarScanViewer.getRoot();
      lidarScanRootNode.setMouseTransparent(true);
      Node stereoVisionPointCloudRootNode = stereoVisionPointCloudViewer.getRoot();
      stereoVisionPointCloudRootNode.setMouseTransparent(true);
      Node detectedObjectRootNode = detectedObjectViewer.getRoot();
      detectedObjectRootNode.setMouseTransparent(true);

      root.getChildren().addAll(lidarScanRootNode, stereoVisionPointCloudRootNode, detectedObjectRootNode, planarRegionMeshView, rawSuperPixelMeshView,
                                fusedSuperPixelMeshView);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            lidarScanViewer.render();
            stereoVisionPointCloudViewer.render();
            detectedObjectViewer.render();

            if (planarRegionsMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(planarRegionMeshView, planarRegionsMeshBuilder.pollMeshAndMaterial());
            if (fusedPixelImageMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(fusedSuperPixelMeshView, fusedPixelImageMeshBuilder.pollMeshAndMaterial());
            if (rawPixelImageBuilder.hasNewMeshAndMaterial())
               updateMeshView(rawSuperPixelMeshView, rawPixelImageBuilder.pollMeshAndMaterial());

            fusedSuperPixelMeshView.setVisible(showFusedSuperPixelData.get());
            rawSuperPixelMeshView.setVisible(showRawSuperPixelData.get());
            planarRegionMeshView.setVisible(showPlanarRegions.get());
         }
      };
      start();
   }
   
   public void clear()
   {
      reaMessager.submitMessageInternal(REAModuleAPI.PlanarRegionsPolygonizerClear, true);
   }

   public void start()
   {
      renderMeshAnimation.start();

      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(lidarScanViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(stereoVisionPointCloudViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(planarRegionsMeshBuilder, 0, SLOW_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(fusedPixelImageMeshBuilder, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(rawPixelImageBuilder, 0, SLOW_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
   }

   public void sleep()
   {
      renderMeshAnimation.stop();
      meshBuilderScheduledFutures.clear();
   }

   public void stop()
   {
      sleep();
   }

   private void updateMeshView(MeshView meshViewToUpdate, Pair<Mesh, Material> meshMaterial)
   {
      meshViewToUpdate.setMesh(meshMaterial.getKey());
      meshViewToUpdate.setMaterial(meshMaterial.getValue());
   }

   public Node getRoot()
   {
      return root;
   }
}