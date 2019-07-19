package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StepUpPlannerErrorMessage" defined in "StepUpPlannerErrorMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StepUpPlannerErrorMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StepUpPlannerErrorMessage_.idl instead.
*
*/
public class StepUpPlannerErrorMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StepUpPlannerErrorMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StepUpPlannerErrorMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StepUpPlannerErrorMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StepUpPlannerErrorMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerErrorMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StepUpPlannerErrorMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getErrorDescription().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StepUpPlannerErrorMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getErrorDescription().length() <= 255)
      cdr.write_type_d(data.getErrorDescription());else
          throw new RuntimeException("error_description field exceeds the maximum length");

      cdr.write_type_9(data.getErrorCode());

      cdr.write_type_4(data.getSequenceIdReceived());

   }

   public static void read(controller_msgs.msg.dds.StepUpPlannerErrorMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getErrorDescription());	
      data.setErrorCode(cdr.read_type_9());
      	
      data.setSequenceIdReceived(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StepUpPlannerErrorMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("error_description", data.getErrorDescription());
      ser.write_type_9("error_code", data.getErrorCode());
      ser.write_type_4("sequence_id_received", data.getSequenceIdReceived());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StepUpPlannerErrorMessage data)
   {
      ser.read_type_d("error_description", data.getErrorDescription());
      data.setErrorCode(ser.read_type_9("error_code"));
      data.setSequenceIdReceived(ser.read_type_4("sequence_id_received"));
   }

   public static void staticCopy(controller_msgs.msg.dds.StepUpPlannerErrorMessage src, controller_msgs.msg.dds.StepUpPlannerErrorMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StepUpPlannerErrorMessage createData()
   {
      return new controller_msgs.msg.dds.StepUpPlannerErrorMessage();
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
   
   public void serialize(controller_msgs.msg.dds.StepUpPlannerErrorMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StepUpPlannerErrorMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StepUpPlannerErrorMessage src, controller_msgs.msg.dds.StepUpPlannerErrorMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StepUpPlannerErrorMessagePubSubType newInstance()
   {
      return new StepUpPlannerErrorMessagePubSubType();
   }
}
