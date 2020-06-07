package us.ihmc.robotEnvironmentAwareness.ui;

import java.io.File;
import java.io.IOException;

import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.SLAMMeshViewer;
import us.ihmc.robotEnvironmentAwareness.ui.controller.DataExporterAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.NormalEstimationAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.SLAMAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.SensorFrameViewer;

public class SLAMBasedEnvironmentAwarenessUI
{
   private static final String UI_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAUIConfiguration.txt";

   private final BorderPane mainPane;

   private final SLAMMeshViewer ihmcSLAMViewer;
   private final REAUIMessager uiMessager;
   private final SensorFrameViewer<StereoVisionPointCloudMessage> depthFrameViewer;
   private final SensorFrameViewer<StampedPosePacket> pelvisFrameViewer;

   @FXML
   private SLAMAnchorPaneController slamAnchorPaneController;
   @FXML
   private DataExporterAnchorPaneController dataExporterAnchorPaneController;
   @FXML
   private NormalEstimationAnchorPaneController normalEstimationController;

   private final Stage primaryStage;

   private final UIConnectionHandler uiConnectionHandler;

   private final StereoVisionPointCloudDataExporter stereoVisionPointCloudDataExporter;

   private SLAMBasedEnvironmentAwarenessUI(REAUIMessager uiMessager, Stage primaryStage) throws Exception
   {
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      // Client
      this.uiMessager = uiMessager;
      uiMessager.startMessager();

      ihmcSLAMViewer = new SLAMMeshViewer(uiMessager);
      depthFrameViewer = new SensorFrameViewer<StereoVisionPointCloudMessage>(uiMessager,
                                                                              SLAMModuleAPI.DepthPointCloudState,
                                                                              SLAMModuleAPI.UISensorPoseHistoryFrames,
                                                                              SensorFrameViewer.createStereoVisionSensorFrameExtractor(),
                                                                              SLAMModuleAPI.SensorPoseHistoryClear);
      pelvisFrameViewer = new SensorFrameViewer<StampedPosePacket>(uiMessager,
                                                                   SLAMModuleAPI.CustomizedFrameState,
                                                                   SLAMModuleAPI.UISensorPoseHistoryFrames,
                                                                   SensorFrameViewer.createStampedPosePacketSensorFrameExtractor(),
                                                                   SLAMModuleAPI.SensorPoseHistoryClear);
      new PlanarRegionSegmentationDataExporter(uiMessager); // No need to anything with it beside instantiating it.
      new PlanarRegionDataExporter(uiMessager); // No need to anything with it beside instantiating it.
      stereoVisionPointCloudDataExporter = new StereoVisionPointCloudDataExporter(uiMessager);

      initializeControllers(uiMessager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      view3dFactory.addNodeToView(ihmcSLAMViewer.getRoot());
      view3dFactory.addNodeToView(depthFrameViewer.getRoot());
      view3dFactory.addNodeToView(pelvisFrameViewer.getRoot());

      uiConnectionHandler = new UIConnectionHandler(primaryStage, uiMessager, SLAMModuleAPI.RequestEntireModuleState);
      uiConnectionHandler.start();

      uiMessager.notifyModuleMessagerStateListeners();

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void refreshModuleState()
   {
      uiMessager.submitStateRequestToModule(SLAMModuleAPI.RequestEntireModuleState);
   }

   private void initializeControllers(REAUIMessager uiMessager)
   {
      File configurationFile = new File(UI_CONFIGURATION_FILE_NAME);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }

      slamAnchorPaneController.setConfigurationFile(configurationFile);
      slamAnchorPaneController.attachREAMessager(uiMessager);
      slamAnchorPaneController.bindControls();

      normalEstimationController.setNormalEstimationParametersTopic(SLAMModuleAPI.NormalEstimationParameters);
      normalEstimationController.setConfigurationFile(null);// TODO;
      normalEstimationController.attachREAMessager(uiMessager);
      normalEstimationController.bindControls();

      //      dataExporterAnchorPaneController.setConfigurationFile(configurationFile);
      //      dataExporterAnchorPaneController.attachREAMessager(uiMessager);
      //      dataExporterAnchorPaneController.setMainWindow(primaryStage);
      //      dataExporterAnchorPaneController.bindControls();
   }

   public void show() throws IOException
   {
      refreshModuleState();
      primaryStage.show();
   }

   public void stop()
   {
      try
      {
         uiConnectionHandler.stop();
         uiMessager.closeMessager();

         ihmcSLAMViewer.stop();
         depthFrameViewer.stop();
         pelvisFrameViewer.stop();

         stereoVisionPointCloudDataExporter.shutdown();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static SLAMBasedEnvironmentAwarenessUI creatIntraprocessUI(Stage primaryStage) throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(SLAMModuleAPI.API,
                                                                NetworkPorts.SLAM_MODULE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      REAUIMessager uiMessager = new REAUIMessager(moduleMessager);
      return new SLAMBasedEnvironmentAwarenessUI(uiMessager, primaryStage);
   }
}
