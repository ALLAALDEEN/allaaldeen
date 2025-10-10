/*
 *  Smoothed.h
 *  Store and calculate smoothed values from sensors.
 *  Created by Matt Fryer on 2017-11-17.
 *  Licensed under LGPL (free to modify and use as you wish)
 *  Optimized for quadcopter flight controller
 */

#pragma once

#define SMOOTHED_AVERAGE 1
#define SMOOTHED_EXPONENTIAL 2

// A class used to store and calculate the values to be smoothed.
template <typename T>
class Smoothed {
  private:
    byte smoothMode;
    uint16_t smoothReadingsFactor = 10; // The smoothing factor. In average mode, this is the number of readings to average.
    uint16_t smoothReadingsPosition = 0; // Current position in the array
    uint16_t smoothReadingsNum = 0; // Number of readings currently being averaged
    T *smoothReading; // Array of readings
    bool initialized = false; // Track initialization state
    
  public:
    Smoothed();
    ~Smoothed(); // Destructor to clean up when class instance killed
    bool begin (byte smoothMode, uint16_t smoothFactor = 10);
    bool add (T newReading);
    T get ();
    T getLast ();
    bool clear ();
    bool isInitialized() { return initialized; }
};

// Constructor
template <typename T>
Smoothed<T>::Smoothed () {
  smoothReading = nullptr;
  initialized = false;
}

// Destructor
template <typename T>
Smoothed<T>::~Smoothed () {
  if (smoothReading != nullptr) {
    delete[] smoothReading;
    smoothReading = nullptr;
  }
}

// Initialize the array for storing sensor values
template <typename T>
bool Smoothed<T>::begin (byte mode, uint16_t smoothFactor) {
  // Clean up existing array if any
  if (smoothReading != nullptr) {
    delete[] smoothReading;
    smoothReading = nullptr;
  }
  
  smoothMode = mode;
  smoothReadingsFactor = smoothFactor;
  
  switch (smoothMode) {
    case SMOOTHED_AVERAGE : // SMOOTHED_AVERAGE
      smoothReading = new T[smoothReadingsFactor]; // Create the actual array of the required size
      
      // Initialize all the values in the array to zero
      for (int thisReading = 0; thisReading < smoothReadingsFactor; thisReading++) {
        smoothReading[thisReading] = 0;
      }
      
      initialized = true;
      return true;
      break;
      
    case SMOOTHED_EXPONENTIAL : // SMOOTHED_EXPONENTIAL
      smoothReading = new T[2];
      smoothReading[0] = 0;
      smoothReading[1] = 0; // Second value in array used for storing last value added
      
      initialized = true;
      return true;
      break;

    default :
      initialized = false;
      return false;
      break;
  }
}

// Add a value to the array
template <typename T>
bool Smoothed<T>::add (T newReading) {
  if (!initialized) return false;
  
  switch (smoothMode) {
    case SMOOTHED_AVERAGE : // SMOOTHED_AVERAGE
      if(smoothReadingsNum < smoothReadingsFactor) { 
        smoothReadingsNum++; 
      } // Keep record of the number of readings being averaged. This will count up to the array size then stay at that number
       
      smoothReading[smoothReadingsPosition] = newReading; // Add the new value
      
      if (smoothReadingsPosition == (smoothReadingsFactor - 1)) { // If at the end of the array
        smoothReadingsPosition = 0; // Increment to the beginning of the array
      } else {
        smoothReadingsPosition++; // Increment to next array position
      }

      return true;
      break;

    case SMOOTHED_EXPONENTIAL : // SMOOTHED_EXPONENTIAL
      if( smoothReadingsNum == 0 ) {
        smoothReadingsNum++;
        smoothReading[0] = newReading;
      } else {
        // Improved exponential smoothing with better numerical stability
        float alpha = (float)smoothReadingsFactor / 100.0;
        alpha = constrain(alpha, 0.0, 1.0); // Ensure alpha is between 0 and 1
        smoothReading[0] = (T)(alpha * newReading + (1.0 - alpha) * smoothReading[0]);
      }

      smoothReading[1] = newReading; // Update the last value added
      
      return true;
      break;
      
    default :
      return false;
      break;
  }
}

// Get the smoothed result
template <typename T>
T Smoothed<T>::get () {
  if (!initialized) return 0;
  
  switch (smoothMode) {
    case SMOOTHED_AVERAGE : { // SMOOTHED_AVERAGE
      if (smoothReadingsNum == 0) return 0;
      
      T runningTotal = 0;
      // Improved calculation to prevent overflows
      for (int x = 0; x < smoothReadingsNum; x++) {
        runningTotal += smoothReading[x];
      }
      return runningTotal / smoothReadingsNum;
    }
      break;

    case SMOOTHED_EXPONENTIAL : // SMOOTHED_EXPONENTIAL
      return smoothReading[0];
      break;

    default :
      return 0;
      break;
  }
}

// Gets the last result stored
template <typename T>
T Smoothed<T>::getLast () {
  if (!initialized) return 0;
  
  switch (smoothMode) {
    case SMOOTHED_AVERAGE : // SMOOTHED_AVERAGE
      // Just return the last reading
      if (smoothReadingsPosition == 0) {
        return smoothReading[smoothReadingsFactor-1];
      } else {
        return smoothReading[smoothReadingsPosition-1];
      }
      break;

    case SMOOTHED_EXPONENTIAL : // SMOOTHED_EXPONENTIAL
      return smoothReading[1];
      break;

    default :
      return 0;
      break;
  }
}

// Clears all stored values
template <typename T>
bool Smoothed<T>::clear () {
  if (!initialized) return false;
  
  switch (smoothMode) {
    case SMOOTHED_AVERAGE : // SMOOTHED_AVERAGE
      // Reset the counters
      smoothReadingsPosition = 0;
      smoothReadingsNum = 0;
      
      // Set all the values in the array to zero
      for (int thisReading = 0; thisReading < smoothReadingsFactor; thisReading++) {
        smoothReading[thisReading] = 0;
      }
      break;

    case SMOOTHED_EXPONENTIAL : // SMOOTHED_EXPONENTIAL
      smoothReadingsNum = 0;
      smoothReading[0] = 0;
      smoothReading[1] = 0;
      break;

    default :
      return false;
      break;
  }
  
  return true;
}