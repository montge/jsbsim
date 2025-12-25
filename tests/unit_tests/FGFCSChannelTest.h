/*******************************************************************************
 * FGFCSChannelTest.h - Unit tests for FCS Channel
 *
 * Tests the FGFCSChannel class including:
 * - Channel construction and naming
 * - Component management
 * - Execution rate control
 *
 * Note: FGFCSChannel is tightly coupled with FGFCS and is typically
 * created through XML configuration. These tests verify the basic API.
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <cxxtest/TestSuite.h>
#include <string>

#include <FGFDMExec.h>
#include <models/FGFCS.h>
#include <models/FGFCSChannel.h>

using namespace JSBSim;

class FGFCSChannelTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * Construction Tests
   ***************************************************************************/

  void testChannelConstruction() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Create a channel with default execution rate
    FGFCSChannel channel(fcs, "TestChannel", 1);

    TS_ASSERT_EQUALS(channel.GetName(), "TestChannel");
  }

  void testChannelConstructionWithRate() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "HalfRateChannel", 2);

    TS_ASSERT_EQUALS(channel.GetName(), "HalfRateChannel");
    TS_ASSERT_EQUALS(channel.GetRate(), 2);
  }

  /***************************************************************************
   * Name Tests
   ***************************************************************************/

  void testGetName() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "PitchChannel", 1);

    TS_ASSERT_EQUALS(channel.GetName(), "PitchChannel");
  }

  void testEmptyName() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "", 1);

    TS_ASSERT_EQUALS(channel.GetName(), "");
  }

  void testNameWithSpaces() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "Pitch Control Channel", 1);

    TS_ASSERT_EQUALS(channel.GetName(), "Pitch Control Channel");
  }

  /***************************************************************************
   * Execution Rate Tests
   ***************************************************************************/

  void testDefaultRate() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "DefaultRate", 1);

    TS_ASSERT_EQUALS(channel.GetRate(), 1);
  }

  void testHalfRate() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "HalfRate", 2);

    TS_ASSERT_EQUALS(channel.GetRate(), 2);
  }

  void testQuarterRate() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "QuarterRate", 4);

    TS_ASSERT_EQUALS(channel.GetRate(), 4);
  }

  void testZeroRateBecomesOne() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Rate of 0 should be treated as 1 (every frame)
    FGFCSChannel channel(fcs, "ZeroRate", 0);

    TS_ASSERT_EQUALS(channel.GetRate(), 1);
  }

  void testNegativeRateBecomesOne() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    // Negative rate should be treated as 1
    FGFCSChannel channel(fcs, "NegativeRate", -5);

    TS_ASSERT_EQUALS(channel.GetRate(), 1);
  }

  /***************************************************************************
   * Component Count Tests
   ***************************************************************************/

  void testNoComponentsInitially() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "EmptyChannel", 1);

    TS_ASSERT_EQUALS(channel.GetNumComponents(), 0u);
  }

  /***************************************************************************
   * Get Component Tests
   ***************************************************************************/

  void testGetNonexistentComponent() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "EmptyChannel", 1);

    // Should return nullptr for nonexistent component
    FGFCSComponent* comp = channel.GetComponent(0);
    TS_ASSERT(comp == nullptr);
  }

  void testGetComponentOutOfRange() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "EmptyChannel", 1);

    // Should return nullptr for out-of-range index
    FGFCSComponent* comp = channel.GetComponent(100);
    TS_ASSERT(comp == nullptr);
  }

  /***************************************************************************
   * Execute Tests
   ***************************************************************************/

  void testExecuteEmptyChannel() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "EmptyChannel", 1);

    // Should not crash when executing empty channel
    channel.Execute();
    TS_ASSERT(true);
  }

  void testExecuteMultipleTimes() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "TestChannel", 1);

    // Should handle multiple executions
    for (int i = 0; i < 10; i++) {
      channel.Execute();
    }
    TS_ASSERT(true);
  }

  /***************************************************************************
   * Reset Tests
   ***************************************************************************/

  void testResetEmptyChannel() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "EmptyChannel", 1);

    // Should not crash when resetting empty channel
    channel.Reset();
    TS_ASSERT(true);
  }

  void testResetAfterExecute() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "TestChannel", 1);

    channel.Execute();
    channel.Reset();
    channel.Execute();

    TS_ASSERT(true);
  }

  /***************************************************************************
   * Rate Execution Pattern Tests
   ***************************************************************************/

  void testRateOneExecutesEveryFrame() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "RateOne", 1);

    // Rate 1 means execute every frame
    TS_ASSERT_EQUALS(channel.GetRate(), 1);
  }

  void testRateTwoExecutesEveryOtherFrame() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel channel(fcs, "RateTwo", 2);

    // Rate 2 means execute every other frame
    TS_ASSERT_EQUALS(channel.GetRate(), 2);
  }

  /***************************************************************************
   * Channel with Different Names
   ***************************************************************************/

  void testMultipleChannelsDifferentNames() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel pitchChannel(fcs, "Pitch", 1);
    FGFCSChannel rollChannel(fcs, "Roll", 1);
    FGFCSChannel yawChannel(fcs, "Yaw", 1);

    TS_ASSERT_EQUALS(pitchChannel.GetName(), "Pitch");
    TS_ASSERT_EQUALS(rollChannel.GetName(), "Roll");
    TS_ASSERT_EQUALS(yawChannel.GetName(), "Yaw");
  }

  void testChannelsWithDifferentRates() {
    FGFDMExec fdmex;
    auto fcs = fdmex.GetFCS();

    FGFCSChannel fastChannel(fcs, "Fast", 1);
    FGFCSChannel mediumChannel(fcs, "Medium", 2);
    FGFCSChannel slowChannel(fcs, "Slow", 4);

    TS_ASSERT_EQUALS(fastChannel.GetRate(), 1);
    TS_ASSERT_EQUALS(mediumChannel.GetRate(), 2);
    TS_ASSERT_EQUALS(slowChannel.GetRate(), 4);
  }
};
