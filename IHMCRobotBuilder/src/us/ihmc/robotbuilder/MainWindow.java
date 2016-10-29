package us.ihmc.robotbuilder;

import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.geometry.Rectangle2D;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ChoiceDialog;
import javafx.scene.text.Font;
import javafx.stage.FileChooser;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javaslang.control.Option;
import us.ihmc.robotbuilder.gui.JointSettingsHolder;
import us.ihmc.robotbuilder.gui.JointTreeView;
import us.ihmc.robotbuilder.gui.Preview3D;
import us.ihmc.robotbuilder.model.Loader;
import us.ihmc.robotbuilder.util.Tree;
import us.ihmc.robotbuilder.util.TreeFocus;
import us.ihmc.robotbuilder.util.Util;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;

import java.io.File;
import java.util.Optional;
import java.util.function.Function;

import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;

/**
 *
 */
public class MainWindow extends Application
{
   @FXML private JointSettingsHolder jointSettings;

   @FXML private Preview3D view3D;

   @FXML private JointTreeView treeView;

   private Stage stage;

   @Override public void start(Stage primaryStage) throws Exception
   {
      Font.loadFont(getClass().getResource("/fonts/FontAwesome.otf").toExternalForm(), 10);
      Parent root = FXMLLoader.load(getClass().getResource("/main_window.fxml"));

      Screen screen = Screen.getPrimary();
      Rectangle2D bounds = screen.getVisualBounds();

      primaryStage.setX(bounds.getMinX());
      primaryStage.setY(bounds.getMinY());
      primaryStage.setWidth(bounds.getWidth());
      primaryStage.setHeight(bounds.getHeight());
      Scene scene = new Scene(root, bounds.getWidth(), bounds.getHeight());

      this.stage = primaryStage;
      stage.setTitle("IHMC Robot Builder");
      stage.setScene(scene);
      stage.setMaximized(true);
      stage.show();
   }

   /**
    * File -> Open action
    */
   public void onOpen()
   {
      FileChooser chooser = new FileChooser();
      chooser.setTitle("Open Robot Definition File");
      if (System.getProperty("initial.dir") != null)
         chooser.setInitialDirectory(new File(System.getProperty("initial.dir")));
      File file = chooser.showOpenDialog(stage);
      if (file == null)
         return;

      Loader.loadFile(file, options -> Util.runLaterInUI(() -> {
         if (options.isEmpty())
             return Option.none();
         if (options.length() == 1)
             return Option.of(options.get(0));

         // Let the user choose a model to load
         ChoiceDialog<String> dialog = new ChoiceDialog<>(options.get(0), options.toJavaArray(String.class));
         dialog.setTitle("Model Selection");
         dialog.setHeaderText("Please choose a model to open.");
         dialog.setContentText("Model:");
         return Option.ofOptional(dialog.showAndWait());
      })).flatMap(immutableRobotDescription -> Util.runLaterInUI(() -> {
         immutableRobotDescription.peek(description -> {
            Tree<JointDescription> tree = Tree.adapt(description, JointDescription::getChildrenJoints);

            jointSettings.setFocusProperty(treeView.focusProperty());
            functional(treeView.focusProperty())
                  .flatMapOptional(Function.identity())
                  .consume(this::updateUIState);

            functional(view3D.jointTreeProperty())
                  .avoidCycles()
                  .consume(treeView::setFocus);
            updateUIState(tree.getFocus());
         });
         treeView.getSelectionModel().select(0);
         return null;
      })).onFailure(err -> Util.runLaterInUI(() -> {
         Alert alert = new Alert(AlertType.ERROR, "Error loading file: " + err.getMessage(), ButtonType.OK);
         alert.showAndWait();
         err.printStackTrace();
         return null;
      }));
    }

   private void updateUIState(TreeFocus<Tree<JointDescription>> newState)
   {
      treeView.setFocus(Optional.of(newState));
      view3D.jointTreeProperty().setValue(Optional.of(newState));
   }
}
