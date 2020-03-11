package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class PlanPathToLocationBehavior extends AbstractBehavior
{
   //wakeup, request plan
   private final boolean DEBUG = false;
   private boolean planningSuccess = false;

   private FootstepPlanningResult planningResult;

   private PipeLine<BehaviorAction> pipeLine;
   private final YoInteger planId = new YoInteger("planId", registry);

   private FramePose3D goalPose = null;
   private double timeout = 5.0;
   private final SleepBehavior sleepBehavior;
   private final FramePose3D initialStanceFootPose = new FramePose3D();
   private RobotSide initialStanceSide;
   private FootstepDataListMessage footstepDataListMessage;
   private FootstepPlanningToolboxOutputStatus footstepPlanningToolboxOutputStatus;

   protected final ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus> footPlanStatusQueue = new ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus>(2);
   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private final IHMCROS2Publisher<FootstepPlannerParametersPacket> footstepPlannerParametersPublisher;

   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   private FootstepPlannerType footStepPlannerToUse = FootstepPlannerType.A_STAR;

   private boolean assumeFlatGround = false;
   private FootstepPlannerParametersBasics footstepPlannerParameters;

   public PlanPathToLocationBehavior(String robotName, Ros2Node ros2Node, FootstepPlannerParametersBasics footstepPlannerParameters, YoDouble yoTime)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.footstepPlannerParameters = footstepPlannerParameters;
      createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlanningToolboxPubGenerator, footPlanStatusQueue::put);
      createSubscriber(PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator, planarRegions::set);

      toolboxStatePublisher = createPublisher(ToolboxStateMessage.class, footstepPlanningToolboxSubGenerator);
      footstepPlanningRequestPublisher = createPublisher(FootstepPlanningRequestPacket.class, footstepPlanningToolboxSubGenerator);
      footstepPlannerParametersPublisher = createPublisher(FootstepPlannerParametersPacket.class, footstepPlanningToolboxSubGenerator);

      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);
   }

   public void setInputs(FramePose3D goalPose, FramePose3D initialStanceFootPose, RobotSide initialStanceSide, FootstepPlannerType footStepPlannerToUse,
                         boolean assumeFlatGround)
   {
      this.goalPose = goalPose;
      this.assumeFlatGround = assumeFlatGround;
      this.footStepPlannerToUse = footStepPlannerToUse;
      this.initialStanceSide = initialStanceSide;
      this.initialStanceFootPose.setIncludingFrame(initialStanceFootPose);
      this.initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public FootstepDataListMessage getFootStepList()
   {
      return footstepDataListMessage;
   }

   public FootstepPlanningResult getPlanningResult()
   {
      return planningResult;
   }

   public FootstepPlanningToolboxOutputStatus geFootstepPlanningToolboxOutputStatus()
   {
      return footstepPlanningToolboxOutputStatus;
   }

   public void setPlanningTimeout(double timeout)
   {
      this.timeout = timeout;
   }

   private void setupPipeline()
   {

      pipeLine.clearAll();

      BehaviorAction wakeup = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               publishTextToSpeech("PlanPathToLocationBehavior: Telling Planner To Wake Up");
            }
            toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

         }
      };

      BehaviorAction requestPlan = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            if (DEBUG)
            {
               publishTextToSpeech("PlanPathToLocationBehavior: Requesting Plan");
            }

            planId.increment();
            FootstepPlanningRequestPacket request = FootstepPlannerMessageTools.createFootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide,
                                                                                                                    goalPose, footStepPlannerToUse); //  FootstepPlannerType.VIS_GRAPH_WITH_A_STAR);
            request.setTimeout(timeout);
            request.setBestEffortTimeout(timeout);
            request.setAssumeFlatGround(assumeFlatGround);
            if (planarRegions.get() != null)
            {
               request.getPlanarRegionsListMessage().set(planarRegions.get());
            }
            else
            {
               publishTextToSpeech("PlanPathToLocationBehavior: Planar regions are null, Requesting Plan without planar regions");
            }
            request.setPlannerRequestId(planId.getIntegerValue());
            request.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE.ordinal());

            FootstepPlannerParametersPacket plannerParametersPacket = new FootstepPlannerParametersPacket();

            FootstepPlannerMessageTools.copyParametersToPacket(plannerParametersPacket, footstepPlannerParameters);
            plannerParametersPacket.setMaximumStepYaw(0.8);
            footstepPlannerParametersPublisher.publish(plannerParametersPacket);

            footstepPlanningRequestPublisher.publish(request);
         }
      };

      BehaviorAction waitForPlan = new BehaviorAction(sleepBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            sleepBehavior.setSleepTime(timeout);
         }

         @Override
         public boolean isDone()
         {

        	 if(footPlanStatusQueue.isNewPacketAvailable())
        	 {
        		 footstepPlanningToolboxOutputStatus= footPlanStatusQueue.getLatestPacket();
        		 planningResult = FootstepPlanningResult.fromByte(footstepPlanningToolboxOutputStatus.getFootstepPlanningResult());
        	 }
        	 
        	 System.out.println("***********^^^^^^^^^^^^^^^ "+ planningResult);
        	 
            return super.isDone() || (planningResult!=null&&planningResult != FootstepPlanningResult.SOLUTION_DOES_NOT_REACH_GOAL);

         }
      };

      BehaviorAction processPlan = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {

            if (footstepPlanningToolboxOutputStatus!=null)
            {

              // footstepPlanningToolboxOutputStatus = footPlanStatusQueue.getLatestPacket();
              // planningResult = FootstepPlanningResult.fromByte(footstepPlanningToolboxOutputStatus.getFootstepPlanningResult());

               if (planningResult == FootstepPlanningResult.OPTIMAL_SOLUTION || planningResult == FootstepPlanningResult.SUB_OPTIMAL_SOLUTION)
               {
                  planningSuccess = true;
                  footstepDataListMessage = footstepPlanningToolboxOutputStatus.getFootstepDataList();
               }
               else if (planningResult == FootstepPlanningResult.SOLUTION_DOES_NOT_REACH_GOAL)
               {
            	   publishTextToSpeech("PlanPathToLocationBehavior: planner timed out after "+timeout+" seconds");

                   planningSuccess = false;
               }
               else
               {
                  publishTextToSpeech("PlanPathToLocationBehavior: bad plan");

                  planningSuccess = false;
               }
            }
            else
            {
               publishTextToSpeech("PlanPathToLocationBehavior: never head back frm footstep planner");

               planningSuccess = false;
            }
         }
      };

      pipeLine.requestNewStage();

      pipeLine.submitSingleTaskStage(wakeup);
      pipeLine.submitSingleTaskStage(requestPlan);
      pipeLine.submitSingleTaskStage(waitForPlan);
      pipeLine.submitSingleTaskStage(processPlan);

   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   public boolean planSuccess()
   {
      if (planningResult != null)
         return (planningResult == FootstepPlanningResult.OPTIMAL_SOLUTION || planningResult == FootstepPlanningResult.SUB_OPTIMAL_SOLUTION);
      return false;
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
      planningSuccess = false;
      planningResult = null;
      footstepDataListMessage = null;
      footstepPlanningToolboxOutputStatus = null;
      footPlanStatusQueue.clear();
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);

   }

   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public void onBehaviorPaused()
   {

   }

   @Override
   public void onBehaviorResumed()
   {

   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

}