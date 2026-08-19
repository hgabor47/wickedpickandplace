# Plan #5: Non-Blocking Motion Task Implementation

## Problem Statement

The current implementation has a critical flaw: **`moveToBlocking()` blocks the entire motionTask**, preventing button reads during motor movement. This means:
- No emergency stop capability while motors are moving
- No code input possible during sequences
- The system is unresponsive for extended periods (seconds to minutes)

### Current Flow (Problematic)
```
motionTask loop:
  ├─ readButtons()          ← Works here
  └─ runDeliverySequence()
       └─ executeStep()
            └─ moveToBlocking()
                 └─ while(motor running) delay(2)  ← BLOCKS EVERYTHING!
```

---

## Solution Architecture

### Core Concept: State Machine with Non-Blocking Polling

Replace the blocking `while` loop with a **state machine** that tracks motion progress and allows periodic button reads.

### New States for Motion Task

| State | Description | Actions |
|-------|-------------|---------|
| `IDLE` | Waiting for code input | Read buttons, process digits |
| `MOVING` | Motors are in transit | Poll motor status, read buttons (emergency only) |
| `MAGNET_WAIT` | Delay before magnet action | Track elapsed time, read buttons |
| `MAGNET_BOOST` | Magnet at 100% duty | Track boost duration |
| `SEQUENCE_WAIT` | User-defined wait between steps | Track elapsed time, read buttons |

---

## Implementation Plan

### Phase 1: Data Structures

#### 1.1 Add State Enum
```cpp
enum MotionTaskState {
  STATE_IDLE,           // Waiting for button input
  STATE_MOVING,         // Motors moving to target
  STATE_MAGNET_WAIT,    // 200ms pre-magnet delay
  STATE_MAGNET_BOOST,   // Magnet boost phase (MAGNET_BOOST_MS)
  STATE_SEQUENCE_WAIT,  // User-defined wait (step.t seconds)
  STATE_PAUSED          // Emergency stop triggered
};
```

#### 1.2 Add Sequence Execution Context
```cpp
struct SequenceContext {
  int cellIndex;              // Which cell sequence
  int stepIndex;              // Current step in sequence
  MotionTaskState state;      // Current sub-state
  
  // Timing context
  unsigned long stateEntryTime;  // When we entered current state
  unsigned long magnetBoostEnd;  // When boost phase ends
  
  // Motion context  
  float targetX, targetY;       // Target coordinates
  float speed;                  // Travel or carry speed
  
  // Flags
  bool emergencyStop;           // Set by emergency button
};

SequenceContext seqCtx;         // Global instance
```

---

### Phase 2: Refactor `motionTask()` to State Machine

#### 2.1 New motionTask Structure
```cpp
void motionTask(void *parameter) {
  for (;;) {
    readButtons();              // ALWAYS run this first
    
    switch (seqCtx.state) {
      case STATE_IDLE:
        handleIdleState();
        break;
        
      case STATE_MOVING:
        handleMovingState();
        break;
        
      case STATE_MAGNET_WAIT:
        handleMagnetWaitState();
        break;
        
      case STATE_MAGNET_BOOST:
        handleMagnetBoostState();
        break;
        
      case STATE_SEQUENCE_WAIT:
        handleSequenceWaitState();
        break;
        
      case STATE_PAUSED:
        handlePausedState();
        break;
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));  // Yield to other tasks
  }
}
```

#### 2.2 State Handler Signatures
```cpp
void handleIdleState();           // Process button input, start sequences
void handleMovingState();         // Check motor completion, transition
void handleMagnetWaitState();     // Wait 200ms, then engage magnet
void handleMagnetBoostState();    // Track boost duration
void handleSequenceWaitState();   // Wait step.t seconds or until button
void handlePausedState();         // Wait for resume/cancel
```

---

### Phase 3: Implement State Handlers

#### 3.1 `handleIdleState()` - Button Processing
```cpp
void handleIdleState() {
  // Buttons already read by readButtons() -> handleDigit()
  // This state just maintains readiness
  
  if (seqCtx.emergencyStop) {
    seqCtx.state = STATE_PAUSED;
    webLog("Emergency stop triggered!");
    motorA->enable(false);
    motorB->enable(false);
    ledcWrite(PWM_CHANNEL, 0);  // Release magnet
  }
}
```

#### 3.2 `handleMovingState()` - Non-Blocking Motion Check
```cpp
void handleMovingState() {
  // Check if motors completed movement
  bool motorsRunning = motorA->isRunning() || motorB->isRunning();
  
  if (!motorsRunning) {
    // Movement complete, proceed to magnet wait phase
    seqCtx.stateEntryTime = millis();
    seqCtx.state = STATE_MAGNET_WAIT;
  }
  
  // Emergency stop check during movement
  if (seqCtx.emergencyStop) {
    motorA->enable(false);
    motorB->enable(false);
    seqCtx.state = STATE_PAUSED;
    webLog("Emergency stop during movement!");
  }
}
```

#### 3.3 `handleMagnetWaitState()` - Pre-Magnet Delay (200ms)
```cpp
void handleMagnetWaitState() {
  unsigned long elapsed = millis() - seqCtx.stateEntryTime;
  
  if (elapsed >= 200) {
    // Proceed to magnet engagement
    const MoveStep &step = cells[seqCtx.cellIndex].steps[seqCtx.stepIndex];
    
    if (step.m == 1) {
      // Engage magnet with boost
      ledcWrite(PWM_CHANNEL, 255);  // 100% duty
      seqCtx.magnetBoostEnd = millis() + MAGNET_BOOST_MS;
      seqCtx.state = STATE_MAGNET_BOOST;
    } else {
      // Release magnet immediately
      ledcWrite(PWM_CHANNEL, 0);
      penDown = false;
      
      // Check if we need sequence wait
      if (step.t == 255) {
        seqCtx.state = STATE_IDLE;
        buttonsActive = true;
        webLog("Sequence complete - infinite wait");
      } else if (step.t > 0) {
        seqCtx.stateEntryTime = millis();
        seqCtx.state = STATE_SEQUENCE_WAIT;
      } else {
        // No wait, proceed to next step
        advanceToNextStep();
      }
    }
  }
  
  // Emergency stop check
  if (seqCtx.emergencyStop) {
    ledcWrite(PWM_CHANNEL, 0);
    motorA->enable(false);
    motorB->enable(false);
    seqCtx.state = STATE_PAUSED;
  }
}
```

#### 3.4 `handleMagnetBoostState()` - Boost Phase Tracking
```cpp
void handleMagnetBoostState() {
  unsigned long elapsed = millis() - seqCtx.magnetBoostEnd;
  
  if (elapsed >= MAGNET_BOOST_MS) {
    // Reduce to hold duty
    ledcWrite(PWM_CHANNEL, MAGNET_HOLD_DUTY);
    penDown = true;
    
    const MoveStep &step = cells[seqCtx.cellIndex].steps[seqCtx.stepIndex];
    
    if (step.t == 255) {
      seqCtx.state = STATE_IDLE;
      buttonsActive = true;
    } else if (step.t > 0) {
      // Calculate remaining wait time (total - magnet_wait - boost)
      long totalWaitMs = step.t * 1000L;
      long usedMs = 200L + MAGNET_BOOST_MS;
      long remainingMs = totalWaitMs - usedMs;
      
      if (remainingMs > 0) {
        seqCtx.stateEntryTime = millis();
        // Store remaining time somewhere... maybe add to SequenceContext?
        seqCtx.state = STATE_SEQUENCE_WAIT;
      } else {
        advanceToNextStep();
      }
    }
  }
}
```

#### 3.5 `handleSequenceWaitState()` - User Wait or Button-Triggered
```cpp
void handleSequenceWaitState() {
  const MoveStep &step = cells[seqCtx.cellIndex].steps[seqCtx.stepIndex];
  
  if (step.t == 255) {
    // Infinite wait: stay here until buttons reactivated
    // Buttons are already active (buttonsActive = true)
    // New code input will transition back to IDLE and start new sequence
  } else {
    unsigned long elapsed = millis() - seqCtx.stateEntryTime;
    unsigned long requiredWait = /* calculate from step.t */;
    
    if (elapsed >= requiredWait) {
      advanceToNextStep();
    }
  }
  
  // Emergency stop check
  if (seqCtx.emergencyStop) {
    ledcWrite(PWM_CHANNEL, 0);
    motorA->enable(false);
    motorB->enable(false);
    seqCtx.state = STATE_PAUSED;
  }
}
```

#### 3.6 `handlePausedState()` - Emergency Recovery
```cpp
void handlePausedState() {
  // Wait for specific button combination to resume or cancel
  // For now: any button press clears emergency flag and returns to IDLE
  
  // TODO: Implement proper pause/resume logic
  // Maybe add a dedicated "RESET" button?
  
  if (/* some condition to clear emergency */) {
    seqCtx.emergencyStop = false;
    motorA->enable(true);
    motorB->enable(true);
    seqCtx.state = STATE_IDLE;
    buttonsActive = true;
    webLog("Emergency cleared, ready for new input");
  }
}
```

---

### Phase 4: Refactor `runDeliverySequence()`

#### 4.1 New Non-Blocking Version
```cpp
void runDeliverySequence(int idx) {
  buttonsActive = false;
  
  seqCtx.cellIndex = idx;
  seqCtx.stepIndex = 0;
  seqCtx.emergencyStop = false;
  seqCtx.state = STATE_MOVING;  // Start with movement
  
  const CellDef &cell = cells[idx];
  const MoveStep &step = cell.steps[0];
  
  seqCtx.targetX = step.x;
  seqCtx.targetY = step.y;
  seqCtx.speed = SPEED_TRAVEL;  // penDown is false initially
  
  webLogf("Cella #%d szekvencia inditasa (%d lepes)", idx, cell.stepCount);
  
  // Start non-blocking motion
  startNonBlockingMove(step.x, step.y, seqCtx.speed);
}

void startNonBlockingMove(float x, float y, float speed) {
  // Calculate target steps (same as moveTo())
  float lenA, lenB, curLenA, curLenB;
  computeStringLengths(x, y, lenA, lenB);
  computeStringLengths(currentX, currentY, curLenA, curLenB);
  
  long targetStepsA = mmToSteps(lenA);
  long targetStepsB = mmToSteps(lenB);
  
  float totalLenDelta = max(abs(lenA - curLenA), abs(lenB - curLenB));
  float durationSec = totalLenDelta / speed;
  if (durationSec <= 0) durationSec = 0.05;
  
  uint32_t speedA = max(1L, (long)(deltaA / durationSec));
  uint32_t speedB = max(1L, (long)(deltaB / durationSec));
  
  motorA->setSpeedInHz(speedA);
  motorB->setSpeedInHz(speedB);
  motorA->setAcceleration(4000);
  motorB->setAcceleration(4000);
  
  // Non-blocking move command
  motorA->move(targetStepsA - motorA->getCurrentPosition());
  motorB->move(targetStepsB - motorB->getCurrentPosition());
  
  // Update position tracking (will be exact when movement completes)
  currentX = x;
  currentY = y;
}
```

#### 4.2 Helper: Advance to Next Step
```cpp
void advanceToNextStep() {
  seqCtx.stepIndex++;
  
  if (seqCtx.stepIndex >= cells[seqCtx.cellIndex].stepCount) {
    // Sequence complete
    webLog("Kesz, varakozas a kovetkezo kodra.");
    buttonsActive = true;
    seqCtx.state = STATE_IDLE;
    return;
  }
  
  const MoveStep &nextStep = cells[seqCtx.cellIndex].steps[seqCtx.stepIndex];
  
  // Determine speed for next move (carry vs travel)
  seqCtx.speed = penDown ? SPEED_CARRY : SPEED_TRAVEL;
  
  // Start next movement immediately
  startNonBlockingMove(nextStep.x, nextStep.y, seqCtx.speed);
  seqCtx.state = STATE_MOVING;
}
```

---

### Phase 5: Emergency Stop Integration

#### 5.1 Add Emergency Button Pin
```cpp
#define EMERGENCY_STOP_PIN 32  // Use GPIO32 (currently unused)

// In setup():
pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
```

#### 5.2 Modify `readButtons()` to Check Emergency
```cpp
void readButtons() {
  // Emergency stop check FIRST (active-low with pull-up)
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    seqCtx.emergencyStop = true;
    return;  // Exit immediately, state machine will handle it
  }
  
  // Existing button debounce logic...
  static bool stableState[8] = {true,true,true,true,true,true,true,true};
  for (int i = 0; i < 8; i++) {
    // ... existing code
  }
}
```

---

## Testing Checklist

- [ ] Motors move without blocking button reads
- [ ] Emergency stop halts motors immediately in all states
- [ ] Sequence completes correctly after emergency recovery
- [ ] Magnet engages/releases at correct times
- [ ] Wait times (step.t) work correctly
- [ ] Infinite wait (t=255) reactivates buttons properly
- [ ] No memory leaks or stack overflow (check Free RAM)

---

## Migration Notes

1. **Keep `moveTo()`** for future use or other tasks
2. **Deprecate `moveToBlocking()`** but don't delete (might be used elsewhere)
3. **Update `executeStep()`** to be non-blocking or remove it entirely
4. **Add debug logging** in each state transition for troubleshooting

---

## Expected Performance Improvement

| Metric | Before | After |
|--------|--------|-------|
| Button response during motion | ❌ Blocked | ✅ 5ms |
| Emergency stop latency | ❌ N/A | ✅ <10ms |
| System responsiveness | Poor | Excellent |
| Code complexity | Low | Medium |

---

## References

- Original issue: [`moveToBlocking()`](src/main.cpp:227) blocks entire motionTask
- Related: Emergency stop requirement (issue #6)
- ESP32 FreeRTOS task management: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/free_rtos.html
