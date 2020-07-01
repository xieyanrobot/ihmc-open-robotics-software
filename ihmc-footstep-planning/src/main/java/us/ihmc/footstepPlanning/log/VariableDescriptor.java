package us.ihmc.footstepPlanning.log;

import us.ihmc.yoVariables.variable.YoVariableType;

public class VariableDescriptor
{
   private final String name;
   private final YoVariableType type;
   private final String registryName;
   private final String[] enumValues;

   public VariableDescriptor(String name, YoVariableType type, String registryName)
   {
      this(name, type, registryName, null);
   }

   public VariableDescriptor(String name, YoVariableType type, String registryName, String[] enumValues)
   {
      this.name = name;
      this.type = type;
      this.registryName = registryName;
      this.enumValues = enumValues;
   }

   public String getName()
   {
      return name;
   }

   public YoVariableType getType()
   {
      return type;
   }

   public String getRegistryName()
   {
      return registryName;
   }

   public String[] getEnumValues()
   {
      return enumValues;
   }
}
