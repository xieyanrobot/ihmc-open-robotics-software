package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerControlElement" defined in "StepUpPlannerControlElement_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerControlElement_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerControlElement_.idl instead.
*
*/
public class StepUpPlannerControlElementPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerControlElement>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerControlElement_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerControlElement data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerControlElement data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerControlElement data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerControlElement data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.getCdrSerializedSize(data.getCop(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerControlElement data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getMultiplier());

      controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.write(data.getCop(), cdr);
   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerControlElement data, us.ihmc.idl.CDR cdr)
   {
      data.setMultiplier(cdr.read_type_6());
      	
      controller_msgs.msg.dds.StepUpPlannerVector2PubSubType.read(data.getCop(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerControlElement data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("multiplier", data.getMultiplier());
      ser.write_type_a("cop", new controller_msgs.msg.dds.StepUpPlannerVector2PubSubType(), data.getCop());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerControlElement data)
   {
      data.setMultiplier(ser.read_type_6("multiplier"));
      ser.read_type_a("cop", new controller_msgs.msg.dds.StepUpPlannerVector2PubSubType(), data.getCop());

   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerControlElement src, controller_msgs.msg.dds.StepUpPlannerControlElement dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerControlElement createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerControlElement();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerControlElement data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerControlElement data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerControlElement src, controller_msgs.msg.dds.StepUpPlannerControlElement dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerControlElementPubSubType newInstance()
   {
      return new StepUpPlannerControlElementPubSubType();
   }
}
