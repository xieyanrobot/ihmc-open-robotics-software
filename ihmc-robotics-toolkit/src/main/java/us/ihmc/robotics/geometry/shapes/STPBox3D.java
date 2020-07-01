package us.ihmc.robotics.geometry.shapes;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.shapes.STPShape3DTools.Box3DSTPSupportingVertexCalculator;
import us.ihmc.robotics.geometry.shapes.interfaces.STPBox3DBasics;
import us.ihmc.robotics.geometry.shapes.interfaces.STPBox3DReadOnly;

public class STPBox3D implements STPBox3DBasics
{
   private double minimumMargin, maximumMargin;
   private double largeRadius, smallRadius;
   private final Box3D rawBox3D = new Box3D();
   private final Box3DSTPSupportingVertexCalculator supportingVertexCalculator = new Box3DSTPSupportingVertexCalculator();

   /**
    * Creates a 1-by-1-by-1 box 3D.
    */
   public STPBox3D()
   {
      addChangeListener(() -> updateRadii());
      getSize().set(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new box 3D and initializes its size.
    *
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public STPBox3D(double sizeX, double sizeY, double sizeZ)
   {
      this();
      getSize().set(sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its size.
    *
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of size components is negative.
    */
   public STPBox3D(Vector3DReadOnly size)
   {
      this();
      getSize().set(size);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param position    the position of this box. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param sizeX       the size of this box along the x-axis.
    * @param sizeY       the size of this box along the y-axis.
    * @param sizeZ       the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public STPBox3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      this();
      set(position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param position    the position of this box. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param size        the size of this box. Not modified.
    * @throws IllegalArgumentException if any of size components is negative.
    */
   public STPBox3D(Point3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      this();
      set(position, orientation, size);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose  the position and orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public STPBox3D(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      this();
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose the position and orientation of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of size components is negative.
    */
   public STPBox3D(Pose3DReadOnly pose, Vector3DReadOnly size)
   {
      this();
      set(pose, size);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose  the position and orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *                                  negative.
    */
   public STPBox3D(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      this();
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose the position and orientation of this box. Not modified.
    * @param size the size of this box. Not modified.
    * @throws IllegalArgumentException if any of size components is negative.
    */
   public STPBox3D(RigidBodyTransformReadOnly pose, Vector3DReadOnly size)
   {
      this();
      set(pose, size);
   }

   /**
    * Creates a new box 3D identical to {@code other}.
    *
    * @param other the other box to copy. Not modified.
    */
   public STPBox3D(Box3DReadOnly other)
   {
      this();
      set(other);
   }

   /**
    * Creates a new box 3D identical to {@code other}.
    *
    * @param other the other box to copy. Not modified.
    */
   public STPBox3D(STPBox3DReadOnly other)
   {
      this();
      set(other);
   }

   @Override
   public Shape3DPoseBasics getPose()
   {
      return rawBox3D.getPose();
   }

   @Override
   public Vector3DBasics getSize()
   {
      return rawBox3D.getSize();
   }

   @Override
   public double getMinimumMargin()
   {
      return minimumMargin;
   }

   @Override
   public double getMaximumMargin()
   {
      return maximumMargin;
   }

   @Override
   public double getSmallRadius()
   {
      return smallRadius;
   }

   @Override
   public double getLargeRadius()
   {
      return largeRadius;
   }

   @Override
   public void setMargins(double minimumMargin, double maximumMargin)
   {
      if (maximumMargin <= minimumMargin)
         throw new IllegalArgumentException("The maximum margin has to be strictly greater that the minimum margin, max margin: " + maximumMargin
               + ", min margin: " + minimumMargin);
      this.minimumMargin = minimumMargin;
      this.maximumMargin = maximumMargin;
      updateRadii();
   }

   /**
    * <pre>
    * r = h
    *      r^2 - g^2 - 0.25 * l<sub>max</sub>
    * R = ------------------------
    *           2 * (r - g)
    * </pre>
    *
    * where:
    * <ul>
    * <li><tt>R</tt> is {@link #largeRadius}
    * <li><tt>r</tt> is {@link #smallRadius}
    * <li><tt>h</tt> is {@link #minimumMargin}
    * <li><tt>g</tt> is {@link #maximumMargin}
    * <li><tt>l<sub>max</max></tt> is the maximum edge length that needs to be covered by the large
    * bounding sphere.
    * </ul>
    */
   protected void updateRadii()
   {
      if (minimumMargin == 0.0 && maximumMargin == 0.0)
      {
         smallRadius = Double.NaN;
         largeRadius = Double.NaN;
      }
      else
      {
         smallRadius = minimumMargin;
         largeRadius = STPShape3DTools.computeLargeRadiusFromMargins(minimumMargin,
                                                                     maximumMargin,
                                                                     STPShape3DTools.computeBox3DMaximumEdgeLengthSquared(getSize()));
      }
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      return supportingVertexCalculator.getSupportingVertex(rawBox3D, smallRadius, largeRadius, supportDirection, supportingVertexToPack);
   }

   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return rawBox3D.getIntermediateVariableSupplier();
   }

   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      rawBox3D.setIntermediateVariableSupplier(newSupplier);
   }

   @Override
   public STPBox3D copy()
   {
      return new STPBox3D(this);
   }

   @Override
   public BoxPolytope3DView asConvexPolytope()
   {
      return rawBox3D.asConvexPolytope();
   }

   /**
    * Notifies the internal listeners that this shape has changed.
    */
   public void notifyChangeListeners()
   {
      rawBox3D.notifyChangeListeners();
   }

   /**
    * Registers a list of listeners to be notified when this shape changes.
    *
    * @param listeners the listeners to register.
    */
   public void addChangeListeners(List<? extends Shape3DChangeListener> listeners)
   {
      rawBox3D.addChangeListeners(listeners);
   }

   /**
    * Registers a listener to be notified when this shape changes.
    *
    * @param listener the listener to register.
    */
   public void addChangeListener(Shape3DChangeListener listener)
   {
      rawBox3D.addChangeListener(listener);
   }

   /**
    * Removes a previously registered listener.
    * <p>
    * This listener will no longer be notified of changes from this pose.
    * </p>
    *
    * @param listener the listener to remove.
    * @return {@code true} if the listener was removed successful, {@code false} if the listener could
    *         not be found.
    */
   public boolean removeChangeListener(Shape3DChangeListener listener)
   {
      return rawBox3D.removeChangeListener(listener);
   }
}
