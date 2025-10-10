/*
 *  Smoothed.h
 *  Store and calculate smoothed values from sensors.
 *  Created by Matt Fryer on 2017-11-17.
 *  Licensed under LGPL (free to modify and use as you wish)
 *  
 *  Optimized version for flight controller applications
 */

#pragma once

#define SMOOTHED_AVERAGE 1
#define SMOOTHED_EXPONENTIAL 2

// A class used to store and calculate the values to be smoothed.
template <typename T>
class Smoothed {
  private:
    byte smoothMode;
    uint16_t smoothReadingsFactor = 10; // The smoothing factor
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
  smoothMode = mode;
  smoothReadingsFactor = smoothFactor; 
  
  // Clean up existing array if it exists
  if (smoothReading != nullptr) {
    delete[] smoothReading;
    smoothReading = nullptr;
  }
  
  switch (smoothMode) {  
    case SMOOTHED_AVERAGE : // SMOOTHED_AVERAGE
      smoothReading = new T[smoothReadingsFactor];
      if (smoothReading == nullptr) return false; // Memory allocation failed
      
      // Initialize all values to zero
      for (uint16_t i = 0; i < smoothReadingsFactor; i++) {
        smoothReading[i] = 0;
      }
      smoothReadingsNum = 0;
      smoothReadingsPosition = 0;
      initialized = true;
      return true;
      break;
      
    case SMOOTHED_EXPONENTIAL : // SMOOTHED_EXPONENTIAL
      smoothReading = new T[2];
      if (smoothReading == nullptr) return false; // Memory allocation failed
      
      smoothReading[0] = 0; // Smoothed value
      smoothReading[1] = 0; // Last raw value
      smoothReadingsNum = 0;
      initialized = true;
      return true;  
      break;

    default : 
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
      }
       
      smoothReading[smoothReadingsPosition] = newReading;
      
      smoothReadingsPosition++;
      if (smoothReadingsPosition >= smoothReadingsFactor) {
        smoothReadingsPosition = 0;
      }

      return true;
      break;

    case SMOOTHED_EXPONENTIAL : // SMOOTHED_EXPONENTIAL
      if( smoothReadingsNum == 0 ) {
        smoothReadingsNum++;
        smoothReading[0] = newReading;
      } else {
        // Improved exponential smoothing with better precision
        float alpha = (float)smoothReadingsFactor / 100.0;
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
      for (uint16_t x = 0; x < smoothReadingsNum; x++) {
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
      smoothReadingsPosition = 0; 
      smoothReadingsNum = 0; 
      
      for (uint16_t i = 0; i < smoothReadingsFactor; i++) {
        smoothReading[i] = 0;
      }
      return true;
      break;

    case SMOOTHED_EXPONENTIAL : // SMOOTHED_EXPONENTIAL
      smoothReadingsNum = 0;
      smoothReading[0] = 0;
      smoothReading[1] = 0;
      return true;
      break;

    default : 
      return false;
      break;
  }         
}