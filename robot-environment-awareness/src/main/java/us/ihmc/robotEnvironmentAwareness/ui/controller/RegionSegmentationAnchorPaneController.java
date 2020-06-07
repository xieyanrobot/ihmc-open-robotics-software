package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.PlanarRegionSegmentationParametersProperty;

public class RegionSegmentationAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableSegmentationButton;
   @FXML
   private Slider searchRadiusSlider;
   @FXML
   private Slider maxDistanceFromPlaneSlider;
   @FXML
   private Slider maxAngleFromPlaneSlider;
   @FXML
   private Slider minNormalQualitySlider;
   @FXML
   private Slider minRegionSizeSlider;
   @FXML
   private Slider maxStandardDeviationSlider;
   @FXML
   private Slider minVolumicDensitySlider;

   private final PlanarRegionSegmentationParametersProperty planarRegionSegmentationParametersProperty = new PlanarRegionSegmentationParametersProperty(this, "planarRegionSegmentationParameters");

   public RegionSegmentationAnchorPaneController()
   {
   }

   private void setupControls()
   {
      searchRadiusSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxDistanceFromPlaneSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxAngleFromPlaneSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
      minNormalQualitySlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxStandardDeviationSlider.setLabelFormatter(StringConverterTools.rounding(1000.0, 1));
      minVolumicDensitySlider.setLabelFormatter(StringConverterTools.rounding(1.0e-6, 2));
   }

   private Topic<Boolean> planarRegionsSegmentationEnableTopic = REAModuleAPI.PlanarRegionsSegmentationEnable;
   private Topic<Boolean> planarRegionsSegmentationClearTopic = REAModuleAPI.PlanarRegionsSegmentationClear;
   private Topic<Boolean> saveRegionUpdateConfigurationTopic = REAModuleAPI.SaveRegionUpdaterConfiguration;
   private Topic<PlanarRegionSegmentationParameters> planarRegionSegmentationParametersTopic = REAModuleAPI.PlanarRegionsSegmentationParameters;

   public void setPlanarRegionsSegmentationEnableTopic(Topic<Boolean> planarRegionsSegmentationEnableTopic)
   {
      this.planarRegionsSegmentationEnableTopic = planarRegionsSegmentationEnableTopic;
   }

   public void setPlanarRegionsSegmentationClearTopic(Topic<Boolean> planarRegionsSegmentationClearTopic)
   {
      this.planarRegionsSegmentationClearTopic = planarRegionsSegmentationClearTopic;
   }

   public void setSaveRegionUpdateConfigurationTopic(Topic<Boolean> saveRegionUpdateConfigurationTopic)
   {
      this.saveRegionUpdateConfigurationTopic = saveRegionUpdateConfigurationTopic;
   }

   public void setPlanarRegionSegmentationParametersTopic(Topic<PlanarRegionSegmentationParameters> planarRegionSegmentationParametersTopic)
   {
      this.planarRegionSegmentationParametersTopic = planarRegionSegmentationParametersTopic;
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(planarRegionsSegmentationEnableTopic, enableSegmentationButton.selectedProperty());

      planarRegionSegmentationParametersProperty.bindBidirectionalSearchRadius(searchRadiusSlider.valueProperty());
      planarRegionSegmentationParametersProperty.bindBidirectionalMaxDistanceFromPlane(maxDistanceFromPlaneSlider.valueProperty());
      planarRegionSegmentationParametersProperty.bindBidirectionalMaxAngleFromPlane(maxAngleFromPlaneSlider.valueProperty());
      planarRegionSegmentationParametersProperty.bindBidirectionalMinNormalQuality(minNormalQualitySlider.valueProperty());
      planarRegionSegmentationParametersProperty.bindBidirectionalMinRegionSize(minRegionSizeSlider.valueProperty());
      planarRegionSegmentationParametersProperty.bindBidirectionalMaxStandardDeviation(maxStandardDeviationSlider.valueProperty());
      planarRegionSegmentationParametersProperty.bindBidirectionalMinVolumicDensity(minVolumicDensitySlider.valueProperty());
      uiMessager.bindBidirectionalGlobal(planarRegionSegmentationParametersTopic, planarRegionSegmentationParametersProperty);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(saveRegionUpdateConfigurationTopic);
   }

   @FXML
   public void clear()
   {
      uiMessager.submitMessageToModule(planarRegionsSegmentationClearTopic, true);
   }
}
