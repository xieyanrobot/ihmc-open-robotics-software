package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * @author Twan Koolen
 */
public class FrameLineSegment2d extends AbstractFrameObject<FrameLineSegment2d, LineSegment2d>
{
   protected final LineSegment2d lineSegment;

   private final Point2D tempPoint2d = new Point2D();
   private final Vector2D tempVector2d = new Vector2D();

   public FrameLineSegment2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameLineSegment2d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new LineSegment2d());
      
      this.lineSegment = this.getGeometryObject();
   }

   public FrameLineSegment2d(ReferenceFrame referenceFrame, LineSegment2d lineSegment2d)
   {
      super(referenceFrame, lineSegment2d);
      
      this.lineSegment = this.getGeometryObject();
   }

   public FrameLineSegment2d(ReferenceFrame referenceFrame, Point2DReadOnly[] endpoints)
   {
      this(referenceFrame);
      this.lineSegment.set(endpoints);
   }

   public FrameLineSegment2d(ReferenceFrame referenceFrame, Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      this(referenceFrame, new LineSegment2d(firstEndpoint, secondEndpoint));
   }

   public FrameLineSegment2d(FramePoint2d[] endpoints)
   {
      this(endpoints[0].getReferenceFrame(), new LineSegment2d(endpoints[0].getPointCopy(), endpoints[1].getPointCopy()));
      
      endpoints[0].checkReferenceFrameMatch(endpoints[1]);
   }

   public FrameLineSegment2d(FramePoint2d firstEndpoint, FramePoint2d secondEndpoint)
   {
      this(firstEndpoint.getReferenceFrame(), new LineSegment2d(firstEndpoint.getPointCopy(), secondEndpoint.getPointCopy()));
      firstEndpoint.checkReferenceFrameMatch(secondEndpoint);
   }

   public FrameLineSegment2d(FrameLineSegment2d frameLineSegment2d)
   {
      this(frameLineSegment2d.getReferenceFrame(), new LineSegment2d(frameLineSegment2d.lineSegment));
   }

   public void set(FramePoint2d firstEndpoint, FramePoint2d secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(secondEndpoint);
      this.lineSegment.set(firstEndpoint.getPoint(), secondEndpoint.getPoint());
   }

   public void setByProjectionOntoXYPlane(FramePoint firstEndpoint, FramePoint secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(secondEndpoint);
      
      this.lineSegment.set(firstEndpoint.getX(), firstEndpoint.getY(), secondEndpoint.getX(), secondEndpoint.getY());
   }

   public void set(FramePoint2d firstEndpoint, FrameVector2d vectorToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(vectorToSecondEndpoint);
      this.lineSegment.set(firstEndpoint.getPoint(), vectorToSecondEndpoint.getVector());
   }

   public void setIncludingFrame(FramePoint2d firstEndpoint, FramePoint2d secondEndpoint)
   {
      this.referenceFrame = firstEndpoint.getReferenceFrame();
      set(firstEndpoint, secondEndpoint);
   }

   public void setIncludingFrame(FramePoint2d[] endpoints)
   {
      setIncludingFrame(endpoints[0], endpoints[1]);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      this.referenceFrame = referenceFrame;
      this.lineSegment.set(firstEndpoint, secondEndpoint);
   }

   public void setFirstEndpoint(FramePoint2d firstEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(referenceFrame);
      lineSegment.set(firstEndpoint.getPoint(), lineSegment.getSecondEndpoint());
   }

   public void setSecondEndpoint(FramePoint2d secondEndpoint)
   {
      secondEndpoint.checkReferenceFrameMatch(referenceFrame);
      lineSegment.set(lineSegment.getFirstEndpoint(), secondEndpoint.getPoint());
   }

   public void setFirstEndpoint(ReferenceFrame referenceFrame, Point2DReadOnly firstEndPoint)
   {
      this.referenceFrame.checkReferenceFrameMatch(referenceFrame);
      lineSegment.set(firstEndPoint, lineSegment.getSecondEndpoint());
   }

   public void setSecondEndpoint(ReferenceFrame referenceFrame, Point2DReadOnly secondEndPoint)
   {
      this.referenceFrame.checkReferenceFrameMatch(referenceFrame);
      lineSegment.set(lineSegment.getFirstEndpoint(), secondEndPoint);
   }

   public void setFirstEndpoint(ReferenceFrame referenceFrame, double firstPointX, double firstPointY)
   {
      this.referenceFrame.checkReferenceFrameMatch(referenceFrame);
      lineSegment.setFirstEndpoint(firstPointX, firstPointY);
   }

   public void setSecondEndpoint(ReferenceFrame referenceFrame, double secondPointX, double secondPointY)
   {
      this.referenceFrame.checkReferenceFrameMatch(referenceFrame);
      lineSegment.setSecondEndpoint(secondPointX, secondPointY);
   }

   public FramePoint2d getFirstEndpointCopy()
   {
      return new FramePoint2d(referenceFrame, lineSegment.getFirstEndpointCopy());
   }

   public FramePoint2d getSecondEndpointCopy()
   {
      return new FramePoint2d(referenceFrame, lineSegment.getSecondEndpointCopy());
   }

   public void getFirstEndpoint(FramePoint2d firstEndpointToPack)
   {
      firstEndpointToPack.setIncludingFrame(referenceFrame, lineSegment.getFirstEndpoint());
   }

   public void getSecondEndpoint(FramePoint2d secondEndpointToPack)
   {
      secondEndpointToPack.setIncludingFrame(referenceFrame, lineSegment.getSecondEndpoint());
   }

   public void getFirstEndpoint(Point2DBasics firstEndpointToPack)
   {
      firstEndpointToPack.set(lineSegment.getFirstEndpoint());
   }

   public void getSecondEndpoint(Point2DBasics secondEndpointToPack)
   {
      secondEndpointToPack.set(lineSegment.getSecondEndpoint());
   }

   public Point2DReadOnly getFirstEndpoint()
   {
      return lineSegment.getFirstEndpoint();
   }

   public Point2DReadOnly getSecondEndpoint()
   {
      return lineSegment.getSecondEndpoint();
   }

   public void getFrameVector(FrameVector2d vectorToPack)
   {
      vectorToPack.setToZero(referenceFrame);
      vectorToPack.sub(lineSegment.getSecondEndpoint(), lineSegment.getFirstEndpoint());
   }

   public void set(FramePoint2d[] endpoints)
   {
      checkReferenceFrameMatch(endpoints[0]);
      checkReferenceFrameMatch(endpoints[1]);

      this.lineSegment.set(endpoints[0].getPoint(), endpoints[1].getPoint());
   }

   public void set(ReferenceFrame referenceFrame, double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      checkReferenceFrameMatch(referenceFrame);
      this.lineSegment.set(firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY);
   }

   @Override
   public void set(FrameLineSegment2d lineSegment)
   {
      checkReferenceFrameMatch(lineSegment);
      this.lineSegment.set(lineSegment.lineSegment);
   }

   // TODO change to setIncludingFrame
   public void setAndChangeFrame(FrameLineSegment2d lineSegment)
   {
      this.referenceFrame = lineSegment.referenceFrame;
      this.lineSegment.set(lineSegment.lineSegment);
   }

   public void set(ReferenceFrame referenceFrame, LineSegment2d lineSegment2d)
   {
      this.referenceFrame = referenceFrame;
      this.lineSegment.set(lineSegment2d);
   }

   public void flipDirection()
   {
      this.lineSegment.flipDirection();
   }

   public LineSegment2d getLineSegment2d()
   {
      return this.lineSegment;
   }

   public LineSegment2d getLineSegment2dCopy()
   {
      return new LineSegment2d(lineSegment);
   }

   public FramePoint2d[] getEndFramePointsCopy()
   {
      FramePoint2d[] endFramePoints = new FramePoint2d[2];
      endFramePoints[0] = new FramePoint2d(referenceFrame, lineSegment.getFirstEndpoint());
      endFramePoints[1] = new FramePoint2d(referenceFrame, lineSegment.getSecondEndpoint());

      return endFramePoints;
   }

   public double length()
   {
      return lineSegment.length();
   }

   public FramePoint2d midpoint()
   {
      double x = (lineSegment.getFirstEndpoint().getX() + lineSegment.getSecondEndpoint().getX()) / 2.0;
      double y = (lineSegment.getFirstEndpoint().getY() + lineSegment.getSecondEndpoint().getY()) / 2.0;

      return new FramePoint2d(referenceFrame, x, y);
   }

   public void midpoint(FramePoint2d midpointToPack)
   {
      midpointToPack.setToZero(referenceFrame);
      midpointToPack.interpolate(lineSegment.getFirstEndpoint(), lineSegment.getSecondEndpoint(), 0.5);
   }

   public double dotProduct(FrameLineSegment2d frameLineSegment2d)
   {
      checkReferenceFrameMatch(frameLineSegment2d);

      return lineSegment.dotProduct(frameLineSegment2d.lineSegment);
   }

   public boolean isBetweenEndpoints(FramePoint2d point2d, double epsilon)
   {
      checkReferenceFrameMatch(point2d);

      return lineSegment.isBetweenEndpoints(point2d.getPoint(), epsilon);
   }

   public double percentageAlongLineSegment(FramePoint2d point2d)
   {
      checkReferenceFrameMatch(point2d);

      return lineSegment.percentageAlongLineSegment(point2d.getPoint());
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      lineSegment.applyTransformAndProjectToXYPlane(transform);
   }

   public FrameLineSegment2d applyTransformCopy(Transform transform)
   {
      FrameLineSegment2d copy = new FrameLineSegment2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   public FrameLineSegment2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      FrameLineSegment2d copy = new FrameLineSegment2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   private RigidBodyTransform temporaryTransformToDesiredFrame;

   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      if (temporaryTransformToDesiredFrame == null)
         temporaryTransformToDesiredFrame = new RigidBodyTransform();

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);

      applyTransformAndProjectToXYPlane(temporaryTransformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   public FrameLineSegment2d changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame)
   {
      FrameLineSegment2d copy = new FrameLineSegment2d(this);
      copy.changeFrameAndProjectToXYPlane(desiredFrame);
      return copy;
   }

   @Override
   public String toString()
   {
      return "" + lineSegment;
   }

   public void orthogonalProjection(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      lineSegment.orthogonalProjection(point.getPoint());
   }

   public void orthogonalProjection(FramePoint2d point, FramePoint2d projectedPointToPack)
   {
      checkReferenceFrameMatch(point);
      projectedPointToPack.setToZero(referenceFrame);
      lineSegment.orthogonalProjection(point.getPoint(), projectedPointToPack.getPoint());
   }

   public FramePoint2d orthogonalProjectionCopy(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      Point2D projected = lineSegment.orthogonalProjectionCopy(point.getPoint());

      return new FramePoint2d(point.getReferenceFrame(), projected);
   }

   public FramePoint2d intersectionWith(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);
      Point2D intersection = this.lineSegment.intersectionWith(line.line);
      if (intersection == null)
      {
         return null;
      }
      return new FramePoint2d(line.getReferenceFrame(), intersection);
   }

   /**
    * Find the intersection point between this FrameLineSegment2d and the argument FrameLine2d and store it in the FramePoint2d argument.
    * @param intersectionPointToPack argument in which the intersect point is stored
    * @param line used to find the intersection with this line segment
    * @return true if successful, false otherwise.
    */
   public boolean intersectionWith(FramePoint2d intersectionPointToPack, FrameLine2d line)
   {
      checkReferenceFrameMatch(line);
      Point2D intersection = this.lineSegment.intersectionWith(line.line);

      if (intersection == null)
         return false;

      intersectionPointToPack.setIncludingFrame(line.getReferenceFrame(), intersection);

      return true;
   }

   public FramePoint2d intersectionWith(FrameLineSegment2d secondLineSegment)
   {
      checkReferenceFrameMatch(secondLineSegment);
      Point2D intersection = this.lineSegment.intersectionWith(secondLineSegment.lineSegment);
      if (intersection == null)
      {
         return null;
      }
      return new FramePoint2d(secondLineSegment.getReferenceFrame(), intersection);
   }

   public FramePoint2d[] intersectionWith(FrameConvexPolygon2d convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);
      Point2D[] intersection = this.lineSegment.intersectionWith(convexPolygon.convexPolygon);
      FramePoint2d[] ret = new FramePoint2d[intersection.length];
      for (int i = 0; i < intersection.length; i++)
      {
         ret[i] = new FramePoint2d(convexPolygon.referenceFrame, intersection[i]);
      }

      return ret;
   }

   public double distance(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return this.lineSegment.distance(point.getPoint());
   }

   public double distance(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);

      return this.lineSegment.distance(line.line);
   }

   public double distance(FrameLineSegment2d secondLineSegment)
   {
      checkReferenceFrameMatch(secondLineSegment);

      return this.lineSegment.distance(secondLineSegment.lineSegment);
   }

   public double distance(FrameConvexPolygon2d convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);

      return this.lineSegment.distance(convexPolygon.convexPolygon);
   }
   
   public boolean isPointOnLeftSideOfLineSegment(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      point.get(tempPoint2d);
      return this.lineSegment.isPointOnLeftSideOfLineSegment(tempPoint2d);
   }

   public boolean isPointOnRightSideOfLineSegment(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return this.lineSegment.isPointOnRightSideOfLineSegment(point.getPointCopy());
   }

   public FrameLineSegment2d shiftToLeftCopy(double distanceToShift)
   {
      return new FrameLineSegment2d(referenceFrame, lineSegment.shiftToLeftCopy(distanceToShift));
   }

   public FrameLineSegment2d shiftToRightCopy(double distanceToShift)
   {
      return new FrameLineSegment2d(referenceFrame, lineSegment.shiftToRightCopy(distanceToShift));
   }

   public void shiftToLeft(double distanceToShift)
   {
      lineSegment.shiftToLeft(distanceToShift);
   }

   public void shiftToRight(double distanceToShift)
   {
      lineSegment.shiftToRight(distanceToShift);
   }

   public FramePoint2d pointBetweenEndPointsGivenParameter(double parameter)
   {
      return new FramePoint2d(referenceFrame, lineSegment.pointBetweenEndPointsGivenParameter(parameter));
   }

   public void pointBetweenEndPointsGivenParameter(FramePoint2d framePoint2dToPack, double parameter)
   {
      framePoint2dToPack.setIncludingFrame(referenceFrame, lineSegment.pointBetweenEndPointsGivenParameter(parameter));
   }
   
   public void getClosestPointOnLineSegment(FramePoint2d framePoint2dToPack, FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      
      lineSegment.getClosestPointOnLineSegment(tempPoint2d, point.getPoint());
      framePoint2dToPack.setIncludingFrame(referenceFrame, tempPoint2d);
   }
   
   public void getPerpendicularBisector(FrameVector2d perpendicularBisectorToPack, double bisectorLengthDesired)
   {
      lineSegment.getPerpendicularBisector(tempVector2d, bisectorLengthDesired);
      perpendicularBisectorToPack.setIncludingFrame(referenceFrame, tempVector2d);
   }

   public static FrameLineSegment2d generateRandomFrameLineSegment2d(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax)
   {
      FramePoint2d randomPoint1 = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);
      FramePoint2d randomPoint2 = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);

      return new FrameLineSegment2d(randomPoint1, randomPoint2);
   }

   @Override
   public void setToNaN()
   {
      lineSegment.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }
}
