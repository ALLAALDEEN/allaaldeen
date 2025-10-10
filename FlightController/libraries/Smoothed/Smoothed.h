/*
 *  Smoothed.h
 *  Store and calculate smoothed values from sensors.
 *  Created by Matt Fryer on 2017-11-17.
 *  Licensed under LGPL (free to modify and use as you wish)
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
  public:
    Smoothed();
    ~Smoothed(); // Destructor to clean up when class instance killed
    bool begin (byte smoothMode, uint16_t smoothFactor = 10);
    bool add (T newReading);
    T get ();
    T getLast ();
    bool clear ();
};

// Constructor
template <typename T>
Smoothed<T>::Smoothed () {
  
}

// Destructor
template <typename T>
Smoothed<T>::~Smoothed () {
  delete[] smoothReading;
}

// Initialize the array for storing sensor values
template <typename T>
bool Smoothed<T>::begin (byte mode, uint16_t smoothFactor) { 
  smoothMode = mode;
  smoothReadingsFactor = smoothFactor; 
  
  switch (smoothMode) {  
    case SMOOTHED_AVERAGE:
      smoothReading = new T[smoothReadingsFactor];
    
      // Initialize all the values in the array to zero
      for (uint16_t thisReading = 0; thisReading < smoothReadingsFactor; thisReading++) {
        smoothReading[thisReading] = 0;
      }

      return true;
      break;
      
    case SMOOTHED_EXPONENTIAL:
      smoothReading = new T[2];
      smoothReading[0] = 0;
      smoothReading[1] = 0; // Second value in array used for storing last value added
      
      return true;  
      break;

    default: 
      return false;
      break;
  }
}

// Add a value to the array
template <typename T>
bool Smoothed<T>::add (T newReading) {
  switch (smoothMode) {    
    case SMOOTHED_AVERAGE:
      if(smoothReadingsNum < smoothReadingsFactor) { 
        smoothReadingsNum++; 
      }
       
      smoothReading[smoothReadingsPosition] = newReading;
      
      if (smoothReadingsPosition == (smoothReadingsFactor - 1)) {
        smoothReadingsPosition = 0;
      } else {
        smoothReadingsPosition++;
      }

      return true;
      break;

    case SMOOTHED_EXPONENTIAL:
      if( smoothReadingsNum == 0 ) {
        smoothReadingsNum++;
        smoothReading[0] = newReading;
      } else {
        // FIX: Improved exponential smoothing formula
        smoothReading[0] = (T)(((float)smoothReadingsFactor/100.0) * newReading + 
                               (1.0 - ((float)smoothReadingsFactor/100.0)) * smoothReading[0]);
      }

      smoothReading[1] = newReading;
      
      return true;
      break;
      
    default: 
      return false;
      break;
  }    
}

// Get the smoothed result
template <typename T>
T Smoothed<T>::get () {
  switch (smoothMode) {
    case SMOOTHED_AVERAGE: {
      // FIX: Improved averaging to prevent overflow
      T runningTotal = 0;
      T tmpRes = 0;
      T remainder = 0;
      
      for (uint16_t x = 0; x < smoothReadingsNum; x++) {
        tmpRes = smoothReading[x] / smoothReadingsNum;
        remainder += smoothReading[x] - tmpRes * smoothReadingsNum;
        runningTotal += tmpRes;
        
        if (remainder >= smoothReadingsNum) {
          tmpRes = remainder / smoothReadingsNum;
          remainder -= tmpRes * smoothReadingsNum;
          runningTotal += tmpRes;
        }
      }
      return runningTotal;
    }
      break;

    case SMOOTHED_EXPONENTIAL:
      return smoothReading[0]; 
      break;

    default: 
      return 0;
      break;
  }   
}

// Gets the last result stored
template <typename T>
T Smoothed<T>::getLast () {
  switch (smoothMode) {  
    case SMOOTHED_AVERAGE:
      if (smoothReadingsPosition == 0) {
        return smoothReading[smoothReadingsFactor-1]; 
      } else {
        return smoothReading[smoothReadingsPosition-1];
      }
      break;

    case SMOOTHED_EXPONENTIAL:
      return smoothReading[1]; 
      break;

    default: 
      return 0;
      break;
  }         
}

// Clears all stored values
template <typename T>
bool Smoothed<T>::clear () {
  switch (smoothMode) {    
    case SMOOTHED_AVERAGE:
      smoothReadingsPosition = 0; 
      smoothReadingsNum = 0; 
      
      for (uint16_t thisReading = 0; thisReading < smoothReadingsFactor; thisReading++) {
        smoothReading[thisReading] = 0;
      }
      return true;
      break;

    case SMOOTHED_EXPONENTIAL:
      smoothReadingsNum = 0;
      smoothReading[0] = 0;
      smoothReading[1] = 0;
      return true;
      break;

    default: 
      return false;
      break;
  }         
}
