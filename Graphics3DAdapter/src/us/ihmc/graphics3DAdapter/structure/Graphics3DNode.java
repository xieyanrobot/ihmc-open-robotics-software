package us.ihmc.graphics3DAdapter.structure;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.ModifierKeyHolder;
import us.ihmc.graphics3DAdapter.NodeType;
import us.ihmc.graphics3DAdapter.SelectedListener;
import us.ihmc.graphics3DAdapter.graphics.LinkGraphics;

public class Graphics3DNode
{
   private final String name;
   private final NodeType nodeType;
   private final Transform3D transform = new Transform3D();
   
   private LinkGraphics graphicsObject;
   private boolean hasGraphicsObjectChanged = false;

   private final ArrayList<Graphics3DNode> childeren = new ArrayList<Graphics3DNode>();
   private final ArrayList<SelectedListener> selectedListeners = new ArrayList<SelectedListener>();
   
   private boolean freezeFrame = false;

   public Graphics3DNode(String name, NodeType nodeType)
   {
      this.name = name;
      this.nodeType = nodeType;
   }

   public synchronized Transform3D getTransform()
   {
      return transform;
   }

   public synchronized void setTransform(Transform3D transform)
   {
      this.transform.set(transform);
   }

   public void addChild(Graphics3DNode child)
   {
      synchronized (childeren)
      {
         childeren.add(child);
      }
   }

   public List<Graphics3DNode> getChildrenNodes()
   {
      synchronized (childeren)
      {
         return Collections.unmodifiableList(childeren);
      }
   }
   
   public synchronized LinkGraphics getGraphicsObjectAndResetHasGraphicsObjectChanged()
   {
      LinkGraphics ret = graphicsObject;
      setHasGraphicsObjectChanged(false);
      return ret;
   }
   
   public synchronized void setHasGraphicsObjectChanged(boolean hasGraphicsObjectChanged)
   {
      this.hasGraphicsObjectChanged = hasGraphicsObjectChanged;
   }
   
   public boolean getHasGraphicsObjectChanged()
   {
      return hasGraphicsObjectChanged;
   }

   public void setGraphicsObject(LinkGraphics graphicsObject)
   {
      this.graphicsObject = graphicsObject;
      setHasGraphicsObjectChanged(true);
   }

   public String getName()
   {
      return name;
   }

   public NodeType getNodeType()
   {
      return nodeType;
   }

   public void notifySelectedListeners(ModifierKeyHolder modifierKeys, Point3d location, Point3d cameraPosition, Vector3d lookAtDirection)
   {
      for(SelectedListener selectedListener : selectedListeners)
      {
         selectedListener.selected(this, modifierKeys, location, cameraPosition, lookAtDirection);
      }
      
      graphicsObject.notifySelectedListeners(this, modifierKeys, location, cameraPosition, lookAtDirection);
   }
   
   public void addSelectedListener(SelectedListener selectedListener)
   {
      selectedListeners.add(selectedListener);
   }
   
   public void setFreezeFrame(boolean freezeFrame)
   {
      this.freezeFrame = freezeFrame;
   }
   
   public boolean getFreezeFrame()
   {
      return freezeFrame;
   }
}
