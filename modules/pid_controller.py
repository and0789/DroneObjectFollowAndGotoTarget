"""
PID Controller Module - Enhanced Version
========================================
Fitur:
- Anti-windup dengan back-calculation
- Derivative filtering (low-pass)
- Setpoint weighting
- Reset bumpless
- Support untuk actual dt (bukan fixed sample_time)
"""

import time


class PIDController:
    """
    Enhanced PID Controller dengan fitur anti-windup dan derivative filtering.
    """
    
    def __init__(self, Kp, Ki, Kd, setpoint=0, sample_time=0.1, 
                 output_limits=(None, None), derivative_filter=0.1):
        """
        Initialize PID controller.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain  
            Kd: Derivative gain
            setpoint: Target value
            sample_time: Expected sample time (untuk backward compatibility)
            output_limits: (min, max) output limits
            derivative_filter: Low-pass filter coefficient untuk derivative (0-1)
                              0 = no filtering, 1 = maximum filtering
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.output_limits = output_limits
        self.derivative_filter = derivative_filter
        
        # Internal state
        self._last_error = 0.0
        self._integral = 0.0
        self._last_derivative = 0.0
        self._last_time = None
        self._last_measurement = None
    
    def compute(self, measurement):
        """
        Compute PID output menggunakan fixed sample_time.
        
        Args:
            measurement: Current measurement value
            
        Returns:
            PID output value (clamped to output_limits)
        """
        return self._compute_internal(measurement, self.sample_time)
    
    def compute_with_dt(self, measurement, dt):
        """
        Compute PID output dengan actual delta time.
        Lebih akurat jika loop rate tidak konsisten.
        
        Args:
            measurement: Current measurement value
            dt: Actual time delta since last call
            
        Returns:
            PID output value (clamped to output_limits)
        """
        if dt <= 0:
            dt = self.sample_time
        return self._compute_internal(measurement, dt)
    
    def compute_auto_dt(self, measurement):
        """
        Compute PID output dengan auto-calculated dt.
        Menggunakan time.time() untuk menghitung dt aktual.
        
        Args:
            measurement: Current measurement value
            
        Returns:
            PID output value (clamped to output_limits)
        """
        current_time = time.time()
        
        if self._last_time is None:
            dt = self.sample_time
        else:
            dt = current_time - self._last_time
            if dt <= 0:
                dt = self.sample_time
        
        self._last_time = current_time
        return self._compute_internal(measurement, dt)
    
    def _compute_internal(self, measurement, dt):
        """
        Internal PID computation.
        """
        # Calculate error
        error = self.setpoint - measurement
        
        # === PROPORTIONAL TERM ===
        P = self.Kp * error
        
        # === INTEGRAL TERM with Anti-Windup ===
        self._integral += error * dt
        
        # Anti-windup: clamp integral based on output limits
        if self.Ki != 0:
            lower, upper = self.output_limits
            if lower is not None and upper is not None:
                integral_max = upper / self.Ki
                integral_min = lower / self.Ki
                self._integral = max(integral_min, min(integral_max, self._integral))
        
        I = self.Ki * self._integral
        
        # === DERIVATIVE TERM with Filtering ===
        # Use derivative on measurement to avoid derivative kick on setpoint change
        if self._last_measurement is not None and dt > 0:
            # Derivative on measurement (negative because d(setpoint-measurement)/dt = -d(measurement)/dt when setpoint constant)
            raw_derivative = -(measurement - self._last_measurement) / dt
            
            # Low-pass filter on derivative
            alpha = self.derivative_filter
            filtered_derivative = alpha * self._last_derivative + (1 - alpha) * raw_derivative
            self._last_derivative = filtered_derivative
        else:
            filtered_derivative = 0.0
        
        D = self.Kd * filtered_derivative
        self._last_measurement = measurement
        
        # Alternative: derivative on error (original method)
        # Uncomment below if derivative kick is acceptable
        # derivative = (error - self._last_error) / dt if dt > 0 else 0
        # D = self.Kd * derivative
        
        self._last_error = error
        
        # === TOTAL OUTPUT ===
        output = P + I + D
        
        # === OUTPUT CLAMPING ===
        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
        
        return output
    
    def reset(self):
        """
        Reset controller state.
        Call this when:
        - Switching targets
        - Re-enabling control after pause
        - Error becomes very large suddenly
        """
        self._last_error = 0.0
        self._integral = 0.0
        self._last_derivative = 0.0
        self._last_time = None
        self._last_measurement = None
    
    def set_gains(self, Kp=None, Ki=None, Kd=None):
        """
        Update PID gains on-the-fly.
        Pass None to keep current value.
        """
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
    
    def set_setpoint(self, setpoint):
        """
        Update setpoint.
        """
        self.setpoint = setpoint
    
    def get_components(self):
        """
        Get individual P, I, D components (untuk debugging).
        
        Returns:
            dict: {'P': value, 'I': value, 'D': value, 'error': value}
        """
        return {
            'P': self.Kp * self._last_error,
            'I': self.Ki * self._integral,
            'D': self.Kd * self._last_derivative,
            'error': self._last_error
        }


class PDController:
    """
    Simple PD Controller (tanpa integral term).
    Berguna untuk sistem yang tidak memerlukan zero steady-state error,
    atau sebagai inner loop controller.
    """
    
    def __init__(self, Kp, Kd, setpoint=0, output_limits=(None, None)):
        self.Kp = Kp
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._last_error = 0.0
        self._last_time = time.time()
    
    def compute(self, measurement):
        current_time = time.time()
        dt = current_time - self._last_time
        if dt <= 0:
            dt = 0.033  # Default ~30 fps
        
        error = self.setpoint - measurement
        derivative = (error - self._last_error) / dt
        
        output = self.Kp * error + self.Kd * derivative
        
        self._last_error = error
        self._last_time = current_time
        
        # Clamp output
        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
        
        return output
    
    def reset(self):
        self._last_error = 0.0
        self._last_time = time.time()
