package us.ihmc.footstepPlanning.ui.controllers;

import us.ihmc.footstepPlanning.log.VariableDescriptor;

import java.util.ArrayList;
import java.util.List;

public class FootstepPlannerVisualizerTools
{
   public static List<VariableDescriptor> search(String searchQuery, List<VariableDescriptor> allDescriptors)
   {
      if (allDescriptors == null)
      {
         return new ArrayList<>();
      }

      // simply implement splitting by '&' for now TODO fancier search if needed

      String[] queryTerms = searchQuery.split("&");
      List<VariableDescriptor> result = new ArrayList<>();
      for (int i = 0; i < allDescriptors.size(); i++)
      {
         String name = allDescriptors.get(i).getName();
         boolean match = true;

         for (int j = 0; j < queryTerms.length; j++)
         {
            if (!name.contains(queryTerms[j]))
            {
               match = false;
               break;
            }
         }

          if (match)
          {
             result.add(allDescriptors.get(i));
          }
      }

      return result;
   }
}
