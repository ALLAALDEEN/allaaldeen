/*
 * Optimized Kalman Filter for Altitude Estimation
 * Enhanced precision and stability
 * 
 * Features:
 * - Improved noise modeling
 * - Better initialization
 * - Enhanced numerical stability
 * - Adaptive filtering
 */

// Kalman filter state structure
struct KalmanState {
  float position;      // Estimated altitude
  float velocity;      // Estimated vertical velocity
  float acceleration;  // Estimated vertical acceleration
};

// Kalman filter covariance matrix (2x2)
struct CovarianceMatrix {
  float p11, p12;  // Position variance and position-velocity covariance
  float p21, p22;  // Velocity-position covariance and velocity variance
};

// Kalman filter parameters
struct KalmanParams {
  float processNoise;     // Process noise variance
  float measurementNoise; // Measurement noise variance
  float dt;              // Time step
  float gravity;         // Gravity constant
};

class KalmanFilter {
private:
  KalmanState state;
  CovarianceMatrix covariance;
  KalmanParams params;
  
  // Adaptive parameters
  float innovationThreshold = 10.0f;  // Threshold for innovation
  float adaptationFactor = 0.1f;      // Adaptation rate
  
  // Numerical stability
  const float MIN_VARIANCE = 1e-6f;
  const float MAX_VARIANCE = 1e6f;
  
public:
  // Constructor
  KalmanFilter() {
    // Initialize state
    state.position = 0.0f;
    state.velocity = 0.0f;
    state.acceleration = 0.0f;
    
    // Initialize covariance
    covariance.p11 = 1.0f;
    covariance.p12 = 0.0f;
    covariance.p21 = 0.0f;
    covariance.p22 = 1.0f;
    
    // Initialize parameters
    params.processNoise = 0.1f;
    params.measurementNoise = 0.5f;
    params.dt = 0.005f;  // 200Hz
    params.gravity = 9.81f;
  }
  
  // Initialize filter
  void initialize(float initialPosition, float initialVelocity = 0.0f) {
    state.position = initialPosition;
    state.velocity = initialVelocity;
    state.acceleration = 0.0f;
    
    // Set initial uncertainty
    covariance.p11 = 1.0f;
    covariance.p12 = 0.0f;
    covariance.p21 = 0.0f;
    covariance.p22 = 1.0f;
  }
  
  // Update filter with new measurement
  float update(float measurement, float dt) {
    params.dt = dt;
    
    // Predict step
    predict();
    
    // Update step
    float innovation = measurement - state.position;
    
    // Adaptive noise estimation
    adaptNoise(innovation);
    
    // Kalman gain
    float S = covariance.p11 + params.measurementNoise;
    float K1 = covariance.p11 / S;
    float K2 = covariance.p21 / S;
    
    // Update state
    state.position += K1 * innovation;
    state.velocity += K2 * innovation;
    
    // Update covariance
    float I_K1 = 1.0f - K1;
    float temp_p11 = covariance.p11 * I_K1;
    float temp_p12 = covariance.p12 * I_K1;
    
    covariance.p11 = temp_p11;
    covariance.p12 = temp_p12;
    covariance.p21 = covariance.p21 - covariance.p11 * K2;
    covariance.p22 = covariance.p22 - covariance.p12 * K2;
    
    // Ensure numerical stability
    ensureStability();
    
    return state.position;
  }
  
  // Get current state
  KalmanState getState() const {
    return state;
  }
  
  // Get position
  float getPosition() const {
    return state.position;
  }
  
  // Get velocity
  float getVelocity() const {
    return state.velocity;
  }
  
  // Get acceleration
  float getAcceleration() const {
    return state.acceleration;
  }
  
  // Set process noise
  void setProcessNoise(float noise) {
    params.processNoise = noise;
  }
  
  // Set measurement noise
  void setMeasurementNoise(float noise) {
    params.measurementNoise = noise;
  }
  
  // Reset filter
  void reset() {
    state.position = 0.0f;
    state.velocity = 0.0f;
    state.acceleration = 0.0f;
    
    covariance.p11 = 1.0f;
    covariance.p12 = 0.0f;
    covariance.p21 = 0.0f;
    covariance.p22 = 1.0f;
  }

private:
  // Prediction step
  void predict() {
    // State transition matrix F
    float F11 = 1.0f;
    float F12 = params.dt;
    float F21 = 0.0f;
    float F22 = 1.0f;
    
    // Process noise matrix Q
    float dt2 = params.dt * params.dt;
    float dt3 = dt2 * params.dt;
    float dt4 = dt3 * params.dt;
    
    float Q11 = params.processNoise * dt4 / 4.0f;
    float Q12 = params.processNoise * dt3 / 2.0f;
    float Q21 = params.processNoise * dt3 / 2.0f;
    float Q22 = params.processNoise * dt2;
    
    // Predict state
    float newPosition = F11 * state.position + F12 * state.velocity;
    float newVelocity = F21 * state.position + F22 * state.velocity;
    
    state.position = newPosition;
    state.velocity = newVelocity;
    
    // Predict covariance
    float new_p11 = F11 * covariance.p11 + F12 * covariance.p21 + Q11;
    float new_p12 = F11 * covariance.p12 + F12 * covariance.p22 + Q12;
    float new_p21 = F21 * covariance.p11 + F22 * covariance.p21 + Q21;
    float new_p22 = F21 * covariance.p12 + F22 * covariance.p22 + Q22;
    
    covariance.p11 = new_p11;
    covariance.p12 = new_p12;
    covariance.p21 = new_p21;
    covariance.p22 = new_p22;
  }
  
  // Adaptive noise estimation
  void adaptNoise(float innovation) {
    float innovationMagnitude = abs(innovation);
    
    if (innovationMagnitude > innovationThreshold) {
      // Increase measurement noise for large innovations
      params.measurementNoise *= (1.0f + adaptationFactor);
    } else {
      // Decrease measurement noise for small innovations
      params.measurementNoise *= (1.0f - adaptationFactor * 0.1f);
    }
    
    // Constrain noise values
    params.measurementNoise = constrain(params.measurementNoise, 0.1f, 10.0f);
  }
  
  // Ensure numerical stability
  void ensureStability() {
    // Constrain covariance values
    covariance.p11 = constrain(covariance.p11, MIN_VARIANCE, MAX_VARIANCE);
    covariance.p22 = constrain(covariance.p22, MIN_VARIANCE, MAX_VARIANCE);
    
    // Ensure positive definiteness
    float det = covariance.p11 * covariance.p22 - covariance.p12 * covariance.p21;
    if (det <= 0.0f) {
      covariance.p11 = MAX(covariance.p11, MIN_VARIANCE);
      covariance.p22 = MAX(covariance.p22, MIN_VARIANCE);
      covariance.p12 = 0.0f;
      covariance.p21 = 0.0f;
    }
  }
};

// Global Kalman filter instance
KalmanFilter altitudeFilter;

// Function prototypes
void initializeKalmanFilter();
float updateAltitudeEstimate(float barometerReading, float dt);

void initializeKalmanFilter() {
  altitudeFilter.initialize(0.0f, 0.0f);
  altitudeFilter.setProcessNoise(0.05f);
  altitudeFilter.setMeasurementNoise(0.3f);
}

float updateAltitudeEstimate(float barometerReading, float dt) {
  return altitudeFilter.update(barometerReading, dt);
}