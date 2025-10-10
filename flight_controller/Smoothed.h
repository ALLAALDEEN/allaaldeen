#pragma once
#include <Arduino.h>

#define SMOOTHED_AVERAGE 1
#define SMOOTHED_EXPONENTIAL 2

template <typename T>
class Smoothed {
 private:
  byte smoothMode;
  uint16_t smoothReadingsFactor = 10;
  uint16_t smoothReadingsPosition = 0;
  uint16_t smoothReadingsNum = 0;
  T *smoothReading;

 public:
  Smoothed();
  ~Smoothed();
  bool begin(byte smoothMode, uint16_t smoothFactor = 10);
  bool add(T newReading);
  T get();
  T getLast();
  bool clear();
};

template <typename T>
Smoothed<T>::Smoothed() {}

template <typename T>
Smoothed<T>::~Smoothed() { delete[] smoothReading; }

template <typename T>
bool Smoothed<T>::begin(byte mode, uint16_t smoothFactor) {
  smoothMode = mode;
  smoothReadingsFactor = smoothFactor;

  switch (smoothMode) {
    case SMOOTHED_AVERAGE:
      smoothReading = new T[smoothReadingsFactor];
      for (int thisReading = 0; thisReading < smoothReadingsNum; thisReading++) {
        smoothReading[thisReading] = 0;
      }
      return true;
    case SMOOTHED_EXPONENTIAL:
      smoothReading = new T[2];
      smoothReading[0] = 0;
      smoothReading[1] = 0;
      return true;
    default:
      return false;
  }
}

template <typename T>
bool Smoothed<T>::add(T newReading) {
  switch (smoothMode) {
    case SMOOTHED_AVERAGE:
      if (smoothReadingsNum < smoothReadingsFactor) {
        smoothReadingsNum++;
      }
      smoothReading[smoothReadingsPosition] = newReading;
      if (smoothReadingsPosition == (smoothReadingsFactor - 1)) {
        smoothReadingsPosition = 0;
      } else {
        smoothReadingsPosition++;
      }
      return true;

    case SMOOTHED_EXPONENTIAL:
      if (smoothReadingsNum == 0) {
        smoothReadingsNum++;
        smoothReading[0] = newReading;
      } else {
        smoothReading[0] = (T)(((long double)smoothReadingsFactor / 100) * newReading + (1 - ((long double)smoothReadingsFactor / 100)) * smoothReading[0]);
      }
      smoothReading[1] = newReading;
      return true;

    default:
      return false;
  }
}

template <typename T>
T Smoothed<T>::get() {
  switch (smoothMode) {
    case SMOOTHED_AVERAGE: {
      T runningTotal = 0;
      T tmpRes = 0;
      T remainder = 0;
      for (int x = 0; x < smoothReadingsNum; x++) {
        tmpRes = smoothReading[x] / smoothReadingsNum;
        remainder += smoothReading[x] - tmpRes * smoothReadingsNum;
        runningTotal += tmpRes;
        if (remainder > smoothReadingsNum) {
          tmpRes = remainder / smoothReadingsNum;
          remainder -= tmpRes * smoothReadingsNum;
          runningTotal += tmpRes;
        }
      }
      return runningTotal;
    }
    case SMOOTHED_EXPONENTIAL:
      return smoothReading[0];
    default:
      return 0;
  }
}

template <typename T>
T Smoothed<T>::getLast() {
  switch (smoothMode) {
    case SMOOTHED_AVERAGE:
      if (smoothReadingsPosition == 0) {
        return smoothReading[smoothReadingsFactor - 1];
      } else {
        return smoothReading[smoothReadingsPosition - 1];
      }
    case SMOOTHED_EXPONENTIAL:
      return smoothReading[1];
    default:
      return 0;
  }
}

template <typename T>
bool Smoothed<T>::clear() {
  switch (smoothMode) {
    case SMOOTHED_AVERAGE:
      smoothReadingsPosition = 0;
      smoothReadingsNum = 0;
      for (int thisReading = 0; thisReading < smoothReadingsNum; thisReading++) {
        smoothReading[thisReading] = 0;
      }
      break;
    case SMOOTHED_EXPONENTIAL:
      smoothReadingsNum = 0;
      smoothReading[0] = 0;
      smoothReading[1] = 0;
      break;
    default:
      return false;
  }
  return true;
}
