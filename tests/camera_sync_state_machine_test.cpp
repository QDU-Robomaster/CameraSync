#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string_view>

#include "CameraSyncStateMachine.hpp"

namespace
{

using CameraSyncDetail::Operation;
using CameraSyncDetail::StateMachine;
using CameraSyncDetail::SyncActions;
using CameraSyncDetail::SyncCommand;
using CameraSyncDetail::SyncEvent;
using CameraSyncDetail::SyncState;

[[noreturn]] void Fail(std::string_view message)
{
  std::cerr << "FAIL: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

void Expect(bool condition, std::string_view message)
{
  if (!condition)
  {
    Fail(message);
  }
}

SyncCommand Stop(uint8_t seq, uint8_t active_level = 1)
{
  return {.operation = Operation::STOP_TRIGGER,
          .active_level = active_level,
          .seq = seq,
          .reserved = 0,
          .trigger_period_us = 0};
}

SyncCommand Start(uint8_t seq, uint32_t period_us, uint8_t active_level = 1)
{
  return {.operation = Operation::START_TRIGGER,
          .active_level = active_level,
          .seq = seq,
          .reserved = 0,
          .trigger_period_us = period_us};
}

void ExpectSingleEvent(const SyncActions& actions, Operation operation, uint8_t seq,
                       uint8_t active_level, uint32_t effective_period_us,
                       uint32_t trigger_sequence, uint64_t timestamp_us,
                       std::string_view context)
{
  if (actions.event_count != 1)
  {
    Fail(context);
  }

  const auto& stamped = actions.events[0];
  Expect(stamped.event.operation == operation, context);
  Expect(stamped.event.seq == seq, context);
  Expect(stamped.event.active_level == active_level, context);
  Expect(stamped.event.reserved == 0, context);
  Expect(stamped.event.effective_period_us == effective_period_us, context);
  Expect(stamped.event.trigger_sequence == trigger_sequence, context);
  Expect(stamped.timestamp_us == timestamp_us, context);
}

void TestWireLayout()
{
  static_assert(sizeof(SyncCommand) == 8);
  static_assert(sizeof(SyncEvent) == 12);
  static_assert(offsetof(SyncCommand, trigger_period_us) == 4);
  static_assert(offsetof(SyncEvent, effective_period_us) == 4);
  static_assert(offsetof(SyncEvent, trigger_sequence) == 8);
  Expect(static_cast<uint8_t>(Operation::STOP_TRIGGER) == 0,
         "STOP_TRIGGER wire value must be 0");
  Expect(static_cast<uint8_t>(Operation::START_TRIGGER) == 1,
         "START_TRIGGER wire value must be 1");
  Expect(static_cast<uint8_t>(Operation::FRAME_TRIGGER) == 2,
         "FRAME_TRIGGER wire value must be 2");
}

void TestDefaultPeriodPublishesEveryRealEdge()
{
  StateMachine machine(10);
  Expect(machine.OnImu(100).event_count == 0,
         "first IMU sample must only establish the default phase");
  Expect(machine.OnImu(109).event_count == 0,
         "default period triggered before its deadline");

  auto actions = machine.OnImu(110);
  Expect(actions.gpio_write_count == 1 && actions.gpio_levels[0] == 1,
         "default deadline must emit an active GPIO edge");
  ExpectSingleEvent(actions, Operation::FRAME_TRIGGER, 0, 1, 10, 1, 110,
                    "default FRAME_TRIGGER contents are wrong");

  actions = machine.OnImu(111);
  Expect(actions.gpio_write_count == 1 && actions.gpio_levels[0] == 0,
         "trigger pulse must return inactive on the next IMU sample");
  Expect(actions.event_count == 0, "pulse release must not publish FRAME_TRIGGER");

  actions = machine.OnImu(120);
  ExpectSingleEvent(actions, Operation::FRAME_TRIGGER, 0, 1, 10, 2, 120,
                    "each default real edge must advance its sequence");
}

void TestLateSampleDoesNotSynthesizeMissedEdges()
{
  StateMachine machine(10);
  machine.OnImu(100);

  auto actions = machine.OnImu(135);
  Expect(actions.gpio_write_count == 1 && actions.gpio_levels[0] == 1,
         "late sample must emit one real GPIO edge");
  ExpectSingleEvent(actions, Operation::FRAME_TRIGGER, 0, 1, 10, 1, 135,
                    "late sample must publish exactly one real edge");
  Expect(machine.PhaseElapsedUs() == 5,
         "late sample must retain the configured period phase");

  actions = machine.OnImu(145);
  Expect(actions.gpio_write_count == 2 && actions.gpio_levels[0] == 0 &&
             actions.gpio_levels[1] == 1,
         "next late sample must release then emit one new edge");
  ExpectSingleEvent(actions, Operation::FRAME_TRIGGER, 0, 1, 10, 2, 145,
                    "missed ideal periods must not inflate real-edge sequence");
  Expect(machine.PhaseElapsedUs() == 5,
         "late cadence must preserve phase without a burst replay");
}

void TestTimestampRollbackRebasesPhase()
{
  StateMachine machine(10);
  machine.OnImu(100);
  machine.OnImu(105);

  auto actions = machine.OnImu(90);
  Expect(actions.gpio_write_count == 0 && actions.event_count == 0,
         "timestamp rollback must not create a trigger edge");
  Expect(machine.PhaseElapsedUs() == 0, "timestamp rollback must clear the old phase");

  Expect(machine.OnImu(99).event_count == 0, "rebased phase triggered too early");
  actions = machine.OnImu(100);
  ExpectSingleEvent(actions, Operation::FRAME_TRIGGER, 0, 1, 10, 1, 100,
                    "rebased phase must trigger after one full period");
}

void TestStopAckCarriesLastRealEdgeSequence()
{
  StateMachine machine(10);
  machine.OnImu(100);
  machine.OnImu(110);
  machine.OnImu(111);
  machine.OnImu(120);

  const SyncCommand stop = Stop(7);
  const auto queued = machine.OnCommand(stop);
  Expect(
      queued.gpio_write_count == 0 && queued.event_count == 0 && machine.CommandPending(),
      "STOP command callback must only queue the command");

  const auto actions = machine.OnImu(121);
  Expect(machine.State() == SyncState::STOPPED,
         "STOP must enter STOPPED on the next IMU sample");
  Expect(!machine.PulseActive(), "STOP must leave no active pulse");
  Expect(actions.gpio_write_count >= 1 &&
             actions.gpio_levels[actions.gpio_write_count - 1] == 0,
         "STOP must leave the GPIO inactive");
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 7, 1, 0, 2, 121,
                    "STOP ACK must carry the last real-edge sequence");

  const auto stopped = machine.OnImu(1000);
  Expect(stopped.gpio_write_count == 0 && stopped.event_count == 0,
         "STOPPED state must not emit camera triggers");
}

void TestStopPreemptsEdgeDueOnSameSample()
{
  StateMachine machine(10);
  machine.OnImu(100);
  machine.OnCommand(Stop(3));

  const auto actions = machine.OnImu(110);
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 3, 1, 0, 0, 110,
                    "STOP must win over an edge due on the same IMU sample");
  Expect(machine.TriggerSequence() == 0, "preempted ideal deadline is not a real edge");
}

void TestStartUsesFreshTimestampPhaseAndSequence()
{
  StateMachine machine(10);
  machine.OnCommand(Stop(1, 0));
  machine.OnImu(100);

  const SyncCommand start = Start(2, 7, 0);
  machine.OnCommand(start);
  auto actions = machine.OnImu(200);
  Expect(machine.State() == SyncState::RUNNING,
         "START must enter RUNNING on the next IMU sample");
  Expect(actions.gpio_write_count == 1 && actions.gpio_levels[0] == 1,
         "low-active START must hold the GPIO inactive");
  ExpectSingleEvent(actions, Operation::START_TRIGGER, 2, 0, 7, 0, 200,
                    "START ACK contents are wrong");

  actions = machine.OnImu(206);
  Expect(actions.gpio_write_count == 0 && actions.event_count == 0,
         "START must wait a full timestamp period before its first edge");
  actions = machine.OnImu(207);
  Expect(actions.gpio_write_count == 1 && actions.gpio_levels[0] == 0,
         "low-active START must emit the configured active edge");
  ExpectSingleEvent(actions, Operation::FRAME_TRIGGER, 2, 0, 7, 1, 207,
                    "first post-START edge must begin at sequence one");
}

void TestStartResetsPreviousRunSequence()
{
  StateMachine machine(10);
  machine.OnImu(100);
  machine.OnImu(110);
  machine.OnImu(120);
  machine.OnCommand(Stop(8));
  auto actions = machine.OnImu(121);
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 8, 1, 0, 2, 121,
                    "pre-START STOP sequence is wrong");

  machine.OnCommand(Start(9, 5));
  actions = machine.OnImu(200);
  ExpectSingleEvent(actions, Operation::START_TRIGGER, 9, 1, 5, 0, 200,
                    "START must reset the real-edge sequence");
  actions = machine.OnImu(205);
  ExpectSingleEvent(actions, Operation::FRAME_TRIGGER, 9, 1, 5, 1, 205,
                    "new run must restart FRAME_TRIGGER sequence at one");
}

void TestDuplicateCommandsReplayOriginalAckOnly()
{
  StateMachine machine(10);
  const SyncCommand stop = Stop(1);
  auto changed_stop = Stop(1, 0);
  changed_stop.trigger_period_us = 123;

  machine.OnCommand(stop);
  machine.OnCommand(changed_stop);
  auto actions = machine.OnImu(100);
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 1, 1, 0, 0, 100,
                    "same pending STOP key must retain original payload");

  actions = machine.OnCommand(changed_stop);
  Expect(actions.gpio_write_count == 0,
         "completed STOP retry must not repeat GPIO effects");
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 1, 1, 0, 0, 100,
                    "completed STOP retry must replay original ACK timestamp");

  const SyncCommand start = Start(2, 10);
  auto changed_start = Start(2, 7, 0);
  changed_start.reserved = 1;
  machine.OnCommand(start);
  machine.OnCommand(changed_start);
  actions = machine.OnImu(200);
  ExpectSingleEvent(actions, Operation::START_TRIGGER, 2, 1, 10, 0, 200,
                    "same pending START key must retain original payload");

  actions = machine.OnImu(210);
  ExpectSingleEvent(actions, Operation::FRAME_TRIGGER, 2, 1, 10, 1, 210,
                    "test precondition must emit a post-START frame");
  actions = machine.OnCommand(changed_start);
  Expect(actions.gpio_write_count == 0,
         "completed START retry must not repeat GPIO effects");
  ExpectSingleEvent(actions, Operation::START_TRIGGER, 2, 1, 10, 0, 200,
                    "FRAME telemetry must not replace completed START ACK");
}

void TestInvalidCommandsAndStatesAreIgnored()
{
  StateMachine machine(10);

  auto invalid = Stop(1);
  invalid.operation = static_cast<Operation>(0xff);
  machine.OnCommand(invalid);
  Expect(!machine.CommandPending(), "unknown operation must be ignored");

  invalid = Stop(1);
  invalid.operation = Operation::FRAME_TRIGGER;
  machine.OnCommand(invalid);
  Expect(!machine.CommandPending(),
         "FRAME_TRIGGER telemetry must not be accepted as a command");

  invalid = Stop(1, 2);
  machine.OnCommand(invalid);
  invalid = Stop(0);
  machine.OnCommand(invalid);
  invalid = Stop(1);
  invalid.reserved = 1;
  machine.OnCommand(invalid);
  invalid = Stop(1);
  invalid.trigger_period_us = 1;
  machine.OnCommand(invalid);
  machine.OnCommand(Start(1, 0));
  Expect(!machine.CommandPending(),
         "invalid level, seq, reserved, and period fields must be ignored");

  machine.OnCommand(Start(2, 10));
  Expect(!machine.CommandPending(), "START is invalid while RUNNING");

  machine.OnCommand(Stop(3));
  machine.OnCommand(Stop(4));
  auto actions = machine.OnImu(100);
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 3, 1, 0, 0, 100,
                    "only the first pending command may execute");
}

void TestPreviousRetryDoesNotBlockNextCommand()
{
  StateMachine machine(10);
  const SyncCommand stop = Stop(1);
  machine.OnCommand(stop);
  machine.OnImu(100);

  auto actions = machine.OnCommand(stop);
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 1, 1, 0, 0, 100,
                    "previous retry must replay before the next command");

  machine.OnCommand(Start(2, 8));
  actions = machine.OnImu(200);
  ExpectSingleEvent(actions, Operation::START_TRIGGER, 2, 1, 8, 0, 200,
                    "next command must still execute exactly once");
}

void TestFreshStopRecoversAfterHostRestart()
{
  StateMachine machine(10);

  machine.OnCommand(Stop(40, 0));
  auto actions = machine.OnImu(100);
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 40, 0, 0, 0, 100,
                    "first host must stop the trigger");

  machine.OnCommand(Stop(1, 1));
  Expect(machine.CommandPending(),
         "a restarted host must be able to queue a fresh STOP while STOPPED");
  actions = machine.OnImu(200);
  Expect(machine.State() == SyncState::STOPPED,
         "fresh STOP must preserve the safe STOPPED state");
  Expect(actions.gpio_write_count == 1 && actions.gpio_levels[0] == 0,
         "fresh STOP must reassert the requested inactive GPIO level");
  ExpectSingleEvent(actions, Operation::STOP_TRIGGER, 1, 1, 0, 0, 200,
                    "fresh STOP must acknowledge the restarted host sequence");

  machine.OnCommand(Start(2, 10));
  actions = machine.OnImu(300);
  ExpectSingleEvent(actions, Operation::START_TRIGGER, 2, 1, 10, 0, 300,
                    "restarted host must be able to START after the fresh STOP ACK");
}

}  // namespace

int main()
{
  TestWireLayout();
  TestDefaultPeriodPublishesEveryRealEdge();
  TestLateSampleDoesNotSynthesizeMissedEdges();
  TestTimestampRollbackRebasesPhase();
  TestStopAckCarriesLastRealEdgeSequence();
  TestStopPreemptsEdgeDueOnSameSample();
  TestStartUsesFreshTimestampPhaseAndSequence();
  TestStartResetsPreviousRunSequence();
  TestDuplicateCommandsReplayOriginalAckOnly();
  TestInvalidCommandsAndStatesAreIgnored();
  TestPreviousRetryDoesNotBlockNextCommand();
  TestFreshStopRecoversAfterHostRestart();
  std::cout << "CameraSync state-machine tests passed\n";
  return EXIT_SUCCESS;
}
