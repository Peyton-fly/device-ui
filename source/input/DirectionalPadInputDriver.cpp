#ifdef INPUTDRIVER_DIRECTIONALPAD_TYPE

#include "input/DirectionalPadInputDriver.h"
#include "Arduino.h"
#include "util/ILog.h"

// Adapted for the master lineage of device-ui: the input-policy pipeline
// (input/policy/*, PR #314) only exists on the input-policy branch, so keys
// are delivered straight to LVGL's keypad protocol instead of being routed
// through an InputPipeline.  Behaviour notes versus the input-policy build:
//   - RETURN (LV_KEY_ESC) is forwarded immediately on press, like any other
//     key; LVGL's native keypad handling fires LV_EVENT_CANCEL.
//   - The driver-side RETURN long-press -> "go home" UICommand 0x101 feature
//     is dropped: there is no UICommandDispatcher on this lineage.

// ---------------------------------------------------------------------------
// Build-flag defaults
// ---------------------------------------------------------------------------
#ifndef INPUTDRIVER_DIRECTIONALPAD_I2C_ADDR
#define INPUTDRIVER_DIRECTIONALPAD_I2C_ADDR 0x22
#endif

#ifndef INPUTDRIVER_DIRECTIONALPAD_WIRE
#define INPUTDRIVER_DIRECTIONALPAD_WIRE Wire
#endif

// ---------------------------------------------------------------------------
// File-scope TCA6424A reader instance
// ---------------------------------------------------------------------------
static Tca6424Pad tca(INPUTDRIVER_DIRECTIONALPAD_I2C_ADDR, INPUTDRIVER_DIRECTIONALPAD_WIRE);

// ---------------------------------------------------------------------------
// Static member definitions
// ---------------------------------------------------------------------------
volatile bool DirectionalPadInputDriver::inputPending = false;
QueueHandle_t DirectionalPadInputDriver::eventQueue = nullptr;
uint8_t DirectionalPadInputDriver::lastPortState = 0xFF; // all bits high = all buttons released
uint32_t DirectionalPadInputDriver::prevKey = 0;

// ---------------------------------------------------------------------------
// PORT 0 bit index → LV_KEY mapping
// ---------------------------------------------------------------------------
uint32_t DirectionalPadInputDriver::portBitToLvKey(uint8_t bit)
{
    switch (bit) {
    case 0:
        return LV_KEY_UP;
    case 1:
        return LV_KEY_DOWN;
    case 2:
        return LV_KEY_LEFT;
    case 3:
        return LV_KEY_RIGHT;
    case 4:
        return LV_KEY_ENTER; // CONFIRM – LVGL fires long-press events automatically
    case 5:
        return LV_KEY_ESC; // RETURN
    default:
        return 0;
    }
}

// ---------------------------------------------------------------------------
// ISR – runs in interrupt context; no I2C access, flag only.
// Placed in IRAM so it is available even when the flash cache is inactive.
// ---------------------------------------------------------------------------
void IRAM_ATTR DirectionalPadInputDriver::intHandler(void)
{
    inputPending = true;
}

// ---------------------------------------------------------------------------
// Constructor / init
// ---------------------------------------------------------------------------
DirectionalPadInputDriver::DirectionalPadInputDriver(void) {}

void DirectionalPadInputDriver::init(void)
{
    eventQueue = xQueueCreate(16, sizeof(PadEvent));

#ifdef TCA6424_REQUIRES_INIT
    if (!tca.begin()) {
        ILOG_WARN("DirectionalPadInputDriver: TCA6424 init failed");
    }
#endif

    // Establish a known baseline before any interrupt fires.
    lastPortState = tca.readPort(0);

    // The TCA6424A INT output is active-low open-drain; enable the internal
    // pull-up so the line idles high and only falls when an input changes.
    pinMode(INPUTDRIVER_DIRECTIONALPAD_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INPUTDRIVER_DIRECTIONALPAD_INT), intHandler, FALLING);

    keyboard = lv_indev_create();
    lv_indev_set_type(keyboard, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(keyboard, button_read);

    if (!inputGroup) {
        inputGroup = lv_group_create();
        lv_group_set_default(inputGroup);
    }
    lv_indev_set_group(keyboard, inputGroup);

    ILOG_DEBUG("DirectionalPadInputDriver: initialised on INT pin %d, I2C addr 0x%02x", INPUTDRIVER_DIRECTIONALPAD_INT,
               INPUTDRIVER_DIRECTIONALPAD_I2C_ADDR);
}

// ---------------------------------------------------------------------------
// LVGL read callback – called by LVGL's input timer on every poll cycle.
//
// Design:
//  1. When the TCA6424A INT fires, the ISR sets inputPending.  Here we do the
//     actual I2C read, diff against lastPortState, and enqueue one PadEvent
//     per changed bit.
//  2. Each poll cycle we drain at most one event from the queue so LVGL
//     processes transitions sequentially.
//  3. If no queued event but a key is still physically held (reflected in
//     lastPortState), we report PRESSED so LVGL's long-press timer keeps
//     running.  No INT fires while the key state is unchanged.
//  4. When the queue is empty and no key is held, we send a final RELEASED for
//     the last active key so LVGL can finalise the click/release event chain.
// ---------------------------------------------------------------------------
void DirectionalPadInputDriver::button_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    // --- Step 1: process pending interrupt -----------------------------------
    if (inputPending) {
        inputPending = false; // clear before reading to avoid losing a second edge
        uint8_t current = tca.readPort(0);
        if (current != 0xFF) { // 0xFF is the I2C-error sentinel; skip if bus failed
            uint8_t changed = current ^ lastPortState;
            for (uint8_t bit = 0; bit < 6; bit++) {
                if (changed & (1u << bit)) {
                    uint32_t key = portBitToLvKey(bit);
                    if (key != 0) {
                        // Active-low: bit = 0 means pressed, bit = 1 means released.
                        bool pressed = !(current & (1u << bit));
                        PadEvent ev{key, pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED};
                        xQueueSend(eventQueue, &ev, 0);
                    }
                }
            }
            lastPortState = current;
        }
    }

    // --- Step 2: deliver one queued transition event --------------------------
    PadEvent e;
    if (xQueueReceive(eventQueue, &e, 0) == pdTRUE) {
        data->key = e.key;
        data->state = e.state;

        // Track the delivered key so steps 3 and 4 report it consistently.
        prevKey = (e.state == LV_INDEV_STATE_PRESSED) ? e.key : 0;
        return;
    }

    // --- Step 3: sustain PRESSED while a key is physically held -------------
    // The INT only fires on *changes*, not while a key is held steady, so we
    // keep reporting PRESSED so LVGL's long-press timer accumulates time.
    uint8_t heldBits = ~lastPortState & 0x3Fu; // bits 0-5 only
    if (heldBits != 0 && prevKey != 0) {
        data->key = prevKey;
        data->state = LV_INDEV_STATE_PRESSED;
        return;
    }

    // --- Step 4: deliver final RELEASED for the last active key -------------
    if (prevKey != 0) {
        data->key = prevKey;
        data->state = LV_INDEV_STATE_RELEASED;
        prevKey = 0;
        return;
    }

    data->state = LV_INDEV_STATE_RELEASED;
}

#endif // INPUTDRIVER_DIRECTIONALPAD_TYPE