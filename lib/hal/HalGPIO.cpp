#include <HalGPIO.h>

#include <Logging.h>
#include <esp_sleep.h>

// Global HalGPIO instance
HalGPIO gpio;

namespace {
constexpr uint16_t TOUCH_SWIPE_THRESHOLD = 70;

uint8_t buttonBit(uint8_t button) { return static_cast<uint8_t>(1U << button); }

uint8_t mapTouchZone(uint16_t x, uint16_t y) {
  if (x >= T5S3_LOGICAL_WIDTH) {
    x = T5S3_LOGICAL_WIDTH - 1;
  }
  if (y >= T5S3_LOGICAL_HEIGHT) {
    y = T5S3_LOGICAL_HEIGHT - 1;
  }

  if (y < 96 && x < 128) {
    return HalGPIO::BTN_BACK;
  }
  if (y < 96 && x > T5S3_LOGICAL_WIDTH - 128) {
    return HalGPIO::BTN_POWER;
  }
  if (y < T5S3_LOGICAL_HEIGHT / 4) {
    return HalGPIO::BTN_UP;
  }
  if (y > (T5S3_LOGICAL_HEIGHT * 3) / 4) {
    return HalGPIO::BTN_DOWN;
  }
  if (x < T5S3_LOGICAL_WIDTH / 3) {
    return HalGPIO::BTN_LEFT;
  }
  if (x > (T5S3_LOGICAL_WIDTH * 2) / 3) {
    return HalGPIO::BTN_RIGHT;
  }
  return HalGPIO::BTN_CONFIRM;
}

uint8_t mapSwipe(uint16_t startX, uint16_t startY, uint16_t x, uint16_t y, uint8_t fallbackButton) {
  const int dx = static_cast<int>(x) - static_cast<int>(startX);
  const int dy = static_cast<int>(y) - static_cast<int>(startY);
  if (abs(dx) < TOUCH_SWIPE_THRESHOLD && abs(dy) < TOUCH_SWIPE_THRESHOLD) {
    return fallbackButton;
  }
  if (abs(dx) >= abs(dy)) {
    return dx > 0 ? HalGPIO::BTN_RIGHT : HalGPIO::BTN_LEFT;
  }
  return dy > 0 ? HalGPIO::BTN_DOWN : HalGPIO::BTN_UP;
}
}  // namespace

void HalGPIO::begin() {
  BoardT5S3::begin();
  const bool touchReady = touch.begin();
  LOG_INF("HW", "Board init: pca9535=%d touch=%d usb=%d", BoardT5S3::pca9535Present(), touchReady,
          BoardT5S3::isUsbConnected());

  lastUsbConnected = isUsbConnected();
  update();
}

uint8_t HalGPIO::readTouchState() {
  BoardT5S3::TouchPoint point;
  if (!touch.readPoint(&point)) {
    touchActive = false;
    return 0;
  }

  uint16_t x = point.x;
  uint16_t y = point.y;
  if (x < T5S3_WIDTH && y < T5S3_HEIGHT && x > T5S3_LOGICAL_WIDTH) {
    const uint16_t logicalX = T5S3_HEIGHT - 1 - y;
    const uint16_t logicalY = x;
    x = logicalX;
    y = logicalY;
  }

  if (!touchActive) {
    touchActive = true;
    touchStartX = x;
    touchStartY = y;
    latchedTouchButton = mapTouchZone(x, y);
  } else {
    latchedTouchButton = mapSwipe(touchStartX, touchStartY, x, y, latchedTouchButton);
  }

  return buttonBit(latchedTouchButton);
}

uint8_t HalGPIO::getState() {
  uint8_t state = 0;

  if (BoardT5S3::readButton()) {
    state |= buttonBit(BTN_CONFIRM);
  }
  if (digitalRead(T5S3_BOOT_BTN) == LOW) {
    state |= buttonBit(BTN_POWER);
  }

  state |= readTouchState();
  return state;
}

void HalGPIO::update() {
  const unsigned long currentTime = millis();
  const uint8_t state = getState();

  pressedEvents = 0;
  releasedEvents = 0;

  if (state != lastState) {
    lastDebounceTime = currentTime;
    lastState = state;
  }

  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (state != currentState) {
      pressedEvents = state & ~currentState;
      releasedEvents = currentState & ~state;

      if (pressedEvents > 0 && currentState == 0) {
        buttonPressStart = currentTime;
      }
      if (releasedEvents > 0 && state == 0) {
        buttonPressFinish = currentTime;
      }

      currentState = state;
    }
  }

  const bool connected = isUsbConnected();
  usbStateChanged = (connected != lastUsbConnected);
  lastUsbConnected = connected;
}

bool HalGPIO::wasUsbStateChanged() const { return usbStateChanged; }

bool HalGPIO::isPressed(uint8_t buttonIndex) const { return currentState & buttonBit(buttonIndex); }

bool HalGPIO::wasPressed(uint8_t buttonIndex) const { return pressedEvents & buttonBit(buttonIndex); }

bool HalGPIO::wasAnyPressed() const { return pressedEvents > 0; }

bool HalGPIO::wasReleased(uint8_t buttonIndex) const { return releasedEvents & buttonBit(buttonIndex); }

bool HalGPIO::wasAnyReleased() const { return releasedEvents > 0; }

unsigned long HalGPIO::getHeldTime() const {
  if (currentState > 0) {
    return millis() - buttonPressStart;
  }
  return buttonPressFinish - buttonPressStart;
}

void HalGPIO::startDeepSleep() {
  while (isPressed(BTN_POWER)) {
    delay(50);
    update();
  }

  BoardT5S3::deinitForSleep();
  pinMode(T5S3_BOOT_BTN, INPUT_PULLUP);
  esp_sleep_enable_ext1_wakeup(1ULL << T5S3_BOOT_BTN, ESP_EXT1_WAKEUP_ANY_LOW);
  esp_deep_sleep_start();
}

void HalGPIO::verifyPowerButtonWakeup(uint16_t requiredDurationMs, bool shortPressAllowed) {
  if (shortPressAllowed) {
    return;
  }

  const uint16_t calibration = millis();
  const uint16_t calibratedDuration = calibration < requiredDurationMs ? requiredDurationMs - calibration : 1;

  const auto start = millis();
  update();
  while (!isPressed(BTN_POWER) && millis() - start < 1000) {
    delay(10);
    update();
  }

  if (isPressed(BTN_POWER)) {
    do {
      delay(10);
      update();
    } while (isPressed(BTN_POWER) && getHeldTime() < calibratedDuration);
    if (getHeldTime() < calibratedDuration) {
      startDeepSleep();
    }
  } else {
    startDeepSleep();
  }
}

bool HalGPIO::isUsbConnected() const { return BoardT5S3::isUsbConnected(); }

HalGPIO::WakeupReason HalGPIO::getWakeupReason() const {
  const auto wakeupCause = esp_sleep_get_wakeup_cause();
  const auto resetReason = esp_reset_reason();
  const bool usbConnected = isUsbConnected();

  if (wakeupCause == ESP_SLEEP_WAKEUP_GPIO || (resetReason == ESP_RST_POWERON && !usbConnected)) {
    return WakeupReason::PowerButton;
  }
  if (wakeupCause == ESP_SLEEP_WAKEUP_UNDEFINED && resetReason == ESP_RST_UNKNOWN && usbConnected) {
    return WakeupReason::AfterFlash;
  }
  if (wakeupCause == ESP_SLEEP_WAKEUP_UNDEFINED && resetReason == ESP_RST_POWERON && usbConnected) {
    return WakeupReason::AfterUSBPower;
  }
  return WakeupReason::Other;
}
