package us.ihmc.humanoidRobotics.communication.networkProcessor;

import static us.ihmc.robotics.Assert.*;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.ConcurrentPacketQueue;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.commons.thread.ThreadTools;

@Disabled
public class NetworkProcessorTest
{
   private static final boolean DEBUG = false;
   private static final IHMCCommunicationKryoNetClassList CLASS_LIST = new IHMCCommunicationKryoNetClassList();

   private enum TestPacketDestinations
   {
      BROADCAST, A, B, C, D, E, F,
   }

   private final Random random = new Random(2641569L);
   
   private final NetworkPorts A = NetworkPorts.createRandomTestPort(random);
   private final NetworkPorts B = NetworkPorts.createRandomTestPort(random);
   private final NetworkPorts C = NetworkPorts.createRandomTestPort(random);
   private final NetworkPorts D = NetworkPorts.createRandomTestPort(random);
   private final NetworkPorts E = NetworkPorts.createRandomTestPort(random);
   private final NetworkPorts F = NetworkPorts.createRandomTestPort(random);
   
   private PacketCommunicator packetCommunicatorAClient = PacketCommunicator.createIntraprocessPacketCommunicator(A, CLASS_LIST);
   private PacketCommunicator packetCommunicatorBClient = PacketCommunicator.createIntraprocessPacketCommunicator(B, CLASS_LIST);
   private PacketCommunicator packetCommunicatorCClient = PacketCommunicator.createIntraprocessPacketCommunicator(C, CLASS_LIST);
   private PacketCommunicator packetCommunicatorDClient = PacketCommunicator.createIntraprocessPacketCommunicator(D, CLASS_LIST);


   private PacketCommunicator packetCommunicatorAServer = PacketCommunicator.createIntraprocessPacketCommunicator(A, CLASS_LIST);
   private PacketCommunicator packetCommunicatorBServer = PacketCommunicator.createIntraprocessPacketCommunicator(B, CLASS_LIST);
   private PacketCommunicator packetCommunicatorCServer = PacketCommunicator.createIntraprocessPacketCommunicator(C, CLASS_LIST);
   private PacketCommunicator packetCommunicatorDServer = PacketCommunicator.createIntraprocessPacketCommunicator(D, CLASS_LIST);
   
   private PacketCommunicator packetCommunicatorEClient = PacketCommunicator.createTCPPacketCommunicatorClient("127.0.0.1", E, CLASS_LIST);
   private PacketCommunicator packetCommunicatorEServer = PacketCommunicator.createTCPPacketCommunicatorServer(E, CLASS_LIST);

   private PacketCommunicator packetCommunicatorFClient = PacketCommunicator.createTCPPacketCommunicatorClient("127.0.0.1", F, CLASS_LIST);
   private PacketCommunicator packetCommunicatorFServer = PacketCommunicator.createTCPPacketCommunicatorServer(F, CLASS_LIST);

   private PacketRouter<TestPacketDestinations> networkProcessor = new PacketRouter<>(TestPacketDestinations.class);

   public void connectCommunicators() throws IOException
   {
      packetCommunicatorAServer.connect();
      packetCommunicatorBServer.connect();
      packetCommunicatorCServer.connect();
      packetCommunicatorDServer.connect();
      packetCommunicatorEServer.connect();
      packetCommunicatorFServer.connect();

      networkProcessor.attachPacketCommunicator(TestPacketDestinations.A, packetCommunicatorAClient);
      networkProcessor.attachPacketCommunicator(TestPacketDestinations.B, packetCommunicatorBClient);
      networkProcessor.attachPacketCommunicator(TestPacketDestinations.C, packetCommunicatorCClient);
      networkProcessor.attachPacketCommunicator(TestPacketDestinations.D, packetCommunicatorDClient);
      networkProcessor.attachPacketCommunicator(TestPacketDestinations.E, packetCommunicatorEClient);
      networkProcessor.attachPacketCommunicator(TestPacketDestinations.F, packetCommunicatorFClient);

      packetCommunicatorAClient.connect();
      packetCommunicatorBClient.connect();
      packetCommunicatorCClient.connect();
      packetCommunicatorDClient.connect();
      packetCommunicatorEClient.connect();
      packetCommunicatorFClient.connect();
   }
   
   public void disconnectCommunicators() throws IOException
   {
      packetCommunicatorAServer.disconnect();
      packetCommunicatorBServer.disconnect();
      packetCommunicatorCServer.disconnect();
      packetCommunicatorDServer.disconnect();
      packetCommunicatorEServer.disconnect();
      packetCommunicatorFServer.disconnect();
      packetCommunicatorAClient.disconnect();
      packetCommunicatorBClient.disconnect();
      packetCommunicatorCClient.disconnect();
      packetCommunicatorDClient.disconnect();
      packetCommunicatorEClient.disconnect();
      packetCommunicatorFClient.disconnect();
      
      networkProcessor.detatchObjectCommunicator(TestPacketDestinations.A);
      networkProcessor.detatchObjectCommunicator(TestPacketDestinations.B);
      networkProcessor.detatchObjectCommunicator(TestPacketDestinations.C);
      networkProcessor.detatchObjectCommunicator(TestPacketDestinations.D);
      networkProcessor.detatchObjectCommunicator(TestPacketDestinations.E);
      networkProcessor.detatchObjectCommunicator(TestPacketDestinations.F);
      
      packetCommunicatorAServer = null;
      packetCommunicatorBServer = null;
      packetCommunicatorCServer = null;
      packetCommunicatorDServer = null;
      packetCommunicatorEServer = null;
      packetCommunicatorFServer = null;
      packetCommunicatorAClient = null;
      packetCommunicatorBClient = null;
      packetCommunicatorCClient = null;
      packetCommunicatorDClient = null;
      packetCommunicatorEClient = null;
      packetCommunicatorFClient = null;
      
      
   }

   @Test
   public void testSendPackets() throws IOException
   {
      connectCommunicators();
      ArrayList<Packet<?>> packetsForB = createRandomPackets(TestPacketDestinations.B);
      ArrayList<ConcurrentPacketQueue<?>> consumersForB = createConsumers(packetsForB, packetCommunicatorBServer);

      sendPackets(packetCommunicatorAServer, packetsForB);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));

      sendPackets(packetCommunicatorBServer, packetsForB);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));

      sendPackets(packetCommunicatorCServer, packetsForB);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));

      sendPackets(packetCommunicatorDServer, packetsForB);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));

      sendPackets(packetCommunicatorEServer, packetsForB);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));

      sendPackets(packetCommunicatorFServer, packetsForB);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));

      ArrayList<Packet<?>> packetsForA = createRandomPackets(TestPacketDestinations.A);
      ArrayList<ConcurrentPacketQueue<?>> consumersForA = createConsumers(packetsForA, packetCommunicatorAServer);
      sendPackets(packetCommunicatorBServer, packetsForA);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForA, consumersForA));

      ArrayList<Packet<?>> packetsForE = createRandomPackets(TestPacketDestinations.E);
      ArrayList<ConcurrentPacketQueue<?>> consumersForE = createConsumers(packetsForE, packetCommunicatorEServer);
      sendPackets(packetCommunicatorBServer, packetsForE);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForE, consumersForE));

      sendPackets(packetCommunicatorEServer, packetsForE);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForE, consumersForE));

      ArrayList<Packet<?>> packetsForF = createRandomPackets(TestPacketDestinations.F);
      ArrayList<ConcurrentPacketQueue<?>> consumersForF = createConsumers(packetsForF, packetCommunicatorFServer);
      sendPackets(packetCommunicatorEServer, packetsForF);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForF, consumersForF));
      
      disconnectCommunicators();
   }

   @Test
   public void testDetatchObjectCommunicator() throws IOException
   {
      connectCommunicators();

      ArrayList<Packet<?>> packetsForA = createRandomPackets(TestPacketDestinations.A);
      ArrayList<Packet<?>> packetsForB = createRandomPackets(TestPacketDestinations.B);
      ArrayList<Packet<?>> packetsForE = createRandomPackets(TestPacketDestinations.E);

      ArrayList<ConcurrentPacketQueue<?>> consumersForA = createConsumers(packetsForA, packetCommunicatorAServer);
      ArrayList<ConcurrentPacketQueue<?>> consumersForB = createConsumers(packetsForB, packetCommunicatorBServer);
      ArrayList<ConcurrentPacketQueue<?>> consumersForE = createConsumers(packetsForE, packetCommunicatorEServer);

      sendPackets(packetCommunicatorAServer, packetsForB);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));

      networkProcessor.detatchObjectCommunicator(TestPacketDestinations.B);

      sendPackets(packetCommunicatorAServer, packetsForB);
      sendPackets(packetCommunicatorBServer, packetsForA);

      assertFalse(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));
      assertFalse(checkIfPacketsGoThroughTheWire(packetsForA, consumersForA));

      sendPackets(packetCommunicatorAServer, packetsForE);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForE, consumersForE));

      networkProcessor.detatchObjectCommunicator(TestPacketDestinations.E);

      sendPackets(packetCommunicatorAServer, packetsForE);
      sendPackets(packetCommunicatorEServer, packetsForA);

      assertFalse(checkIfPacketsGoThroughTheWire(packetsForE, consumersForE));
      assertFalse(checkIfPacketsGoThroughTheWire(packetsForA, consumersForA));
      
      disconnectCommunicators();
   }

   @Test
   public void testForwarder() throws IOException
   {
      connectCommunicators();
      networkProcessor.setPacketRedirects(TestPacketDestinations.A, TestPacketDestinations.B);
      networkProcessor.setPacketRedirects(TestPacketDestinations.E, TestPacketDestinations.C);
      ArrayList<Packet<?>> packetsForA = createRandomPackets(TestPacketDestinations.A);
      ArrayList<Packet<?>> packetsForE = createRandomPackets(TestPacketDestinations.E);

      ArrayList<ConcurrentPacketQueue<?>> consumersForB = createConsumers(packetsForA, packetCommunicatorBServer);
      ArrayList<ConcurrentPacketQueue<?>> consumersForA = createConsumers(packetsForA, packetCommunicatorAServer);
      ArrayList<ConcurrentPacketQueue<?>> consumersForC = createConsumers(packetsForE, packetCommunicatorCServer);
      ArrayList<ConcurrentPacketQueue<?>> consumersForE = createConsumers(packetsForE, packetCommunicatorEServer);

      sendPackets(packetCommunicatorCServer, packetsForA);
      sendPackets(packetCommunicatorDServer, packetsForE);

      assertFalse(checkIfPacketsGoThroughTheWire(packetsForA, consumersForA));
      assertFalse(checkIfPacketsGoThroughTheWire(packetsForE, consumersForE));
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForA, consumersForB));
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForE, consumersForC));
      
      disconnectCommunicators();
   }

   @Test
   public void testDoubleForwarder() throws IOException
   {
      connectCommunicators();
      networkProcessor.setPacketRedirects(TestPacketDestinations.A, TestPacketDestinations.B);
      boolean threwException = false;
      try
      {
         networkProcessor.setPacketRedirects(TestPacketDestinations.B, TestPacketDestinations.C);
      }
      catch(IllegalArgumentException e)
      {
         threwException = true;
      }
      disconnectCommunicators();
      assertTrue(threwException);
   }

   @Test
   public void testDisconnectDoesNotFail() throws IOException
   {
      connectCommunicators();

      ArrayList<Packet<?>> packetsForA = createRandomPackets(TestPacketDestinations.A);
      ArrayList<Packet<?>> packetsForB = createRandomPackets(TestPacketDestinations.B);
      ArrayList<Packet<?>> packetsForE = createRandomPackets(TestPacketDestinations.E);

      ArrayList<ConcurrentPacketQueue<?>> consumersForA = createConsumers(packetsForA, packetCommunicatorAServer);
      ArrayList<ConcurrentPacketQueue<?>> consumersForB = createConsumers(packetsForB, packetCommunicatorBServer);
      ArrayList<ConcurrentPacketQueue<?>> consumersForE = createConsumers(packetsForE, packetCommunicatorEServer);

      sendPackets(packetCommunicatorAServer, packetsForB);
      sendPackets(packetCommunicatorAServer, packetsForE);
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));
      assertTrue(checkIfPacketsGoThroughTheWire(packetsForE, consumersForE));

      packetCommunicatorBClient.disconnect();
      packetCommunicatorEServer.disconnect();

      sendPackets(packetCommunicatorAServer, packetsForB);
      sendPackets(packetCommunicatorBServer, packetsForA);
      sendPackets(packetCommunicatorAServer, packetsForE);

      assertFalse(checkIfPacketsGoThroughTheWire(packetsForB, consumersForB));
      assertFalse(checkIfPacketsGoThroughTheWire(packetsForA, consumersForA));
      assertFalse(checkIfPacketsGoThroughTheWire(packetsForE, consumersForE));
      
      disconnectCommunicators();
   }

   private ArrayList<Packet<?>> createRandomPackets(TestPacketDestinations destination)
   {
      ArrayList<Packet<?>> randomPackets = new ArrayList<Packet<?>>();

      ArrayList<Class<?>> listToPack = new ArrayList<Class<?>>();
      CLASS_LIST.getPacketClassList(listToPack);
      LinkedHashSet<Class<?>> packetsWithoutDuplicates = new LinkedHashSet<>(listToPack);
      listToPack.clear();
      listToPack.addAll(packetsWithoutDuplicates);

      for (Class<?> clazz : listToPack)
      {
         if (Modifier.isAbstract(clazz.getModifiers()) || clazz.isEnum() || clazz.isInterface())
         {
            continue;
         }
         try
         {
            Packet<?> packet = null;
            Constructor<?>[] constructors = clazz.getConstructors();
            for (Constructor<?> constructor : constructors)
            {
               Class<?>[] pType = constructor.getParameterTypes();
               if (pType.length == 1 && pType[0] == Random.class)
               {
                  packet = (Packet<?>) constructor.newInstance(random);
               }
            }
            if (packet == null)
            {
               packet = (Packet<?>) clazz.newInstance();
            }
            packet.setDestination(destination.ordinal());
            randomPackets.add(packet);

         }
         catch (Exception e)
         {
            System.err.println(e);
            e.printStackTrace();
         }
      }
      return randomPackets;
   }

   
   private ArrayList<ConcurrentPacketQueue<?>> createConsumers(ArrayList<Packet<?>> packets, PacketCommunicator communicator)
   {
      ArrayList<ConcurrentPacketQueue<?>> consumers = new ArrayList<ConcurrentPacketQueue<?>>();
      for (int i = 0; i < packets.size(); i++)
      {
         ConcurrentPacketQueue packetQueue = new ConcurrentPacketQueue();
         communicator.attachListener(packets.get(i).getClass(), packetQueue);
         consumers.add(packetQueue);
      }
      return consumers;
   }

   private void sendPackets(PacketCommunicator communicator, ArrayList<Packet<?>> packets)
   {
      for (int i = 0; i < packets.size(); i++)
      {
         communicator.send(packets.get(i));
      }
   }

//   @SuppressWarnings({ "rawtypes", "unchecked" })
   private boolean checkIfPacketsGoThroughTheWire(ArrayList<Packet<?>> packets, ArrayList<ConcurrentPacketQueue<?>> consumers)
   {
      boolean[] receivedStatus = new boolean[consumers.size()];
      long tMax = 500;
      long t = 0;
      long tInc = 10;

      boolean received = false;
      while (!received && t < tMax)
      {
         ThreadTools.sleep(tInc);
         t += tInc;

         for (int i = 0; i < packets.size(); i++)
         {
            ConcurrentPacketQueue<?> listener = consumers.get(i);
            if (listener.isNewPacketAvailable())
            {
               Packet origPacket = packets.get(i);
               Packet recvPacket = listener.getPacket();
               boolean sentCorrectly = origPacket.epsilonEquals(recvPacket, 0.0001);
               receivedStatus[i] = sentCorrectly;
               if (DEBUG)
               {
                  System.out.println(origPacket.getClass().getSimpleName() + "received correctly: " + sentCorrectly);
               }
            }
         }

         received = true;
         for (int i = 0; i < receivedStatus.length; i++)
         {
            received &= receivedStatus[i];
         }
      }
      return received;
   }
}
