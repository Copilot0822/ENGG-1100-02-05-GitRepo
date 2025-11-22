#ifndef CLASSES_H
#define CLASSES_H

#include <Arduino.h>
#include <math.h>

class Switch{
  int pin;
  bool inverted;

  public:
    Switch(int Pin, bool Invert = false){
      pin = Pin;
      inverted = Invert;
      // pinMode(pin, INPUT_PULLUP);
    }
    void init(){
      pinMode(pin, INPUT_PULLUP);
    }
    bool State(){
      if(inverted){
        return digitalRead(pin) == LOW;
      } else {
        return digitalRead(pin) == HIGH;
      }
    }
};

class UltrasonicSensor{
  int trigPin;
  int echoPin;
  long duration;
  float distance;
  
  public:
    UltrasonicSensor(int trigpin, int echopin){
      trigPin = trigpin;
      echoPin = echopin;
    }
    void init(){
      pinMode(echoPin, INPUT);
      pinMode(trigPin, OUTPUT);
    }
    float Distance(){ 
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin,  HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration  = pulseIn(echoPin, HIGH);
      distance= duration*0.034f/2.0f;
      return distance;
    }
};

class L298N{
  int ena;
  int in1;
  int in2;
  bool braking;
  public:
    L298N(int enPin, int in1Pin, int in2Pin, bool brake = true){
      ena = enPin;
      in1 = in1Pin;
      in2 = in2Pin;
      braking = brake;
    }
    void init(){
      pinMode(ena, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
    }
    void setSpeed(double speed = 0){
      if(speed == 0 && braking){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, HIGH);
        digitalWrite(ena, LOW);
      }
      else if(speed == 0 && !braking){
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(ena, LOW);
      }
      else if(speed > 0){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(ena, speed*255);
      }
      else if(speed < 0){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(ena, speed*-255);
      }
    }
};

class MotionMagic7 {
  public:
      MotionMagic7(float kP, float kI, float kD, float kF)
          : _kP(kP), _kI(kI), _kD(kD), _kF(kF) {}

      void setConstraints(float maxVel, float maxAccel, float maxJerk) {
          _maxVel   = (maxVel   > 0) ? maxVel   : 1.0f;
          _maxAccel = (maxAccel > 0) ? maxAccel : 1.0f;
          _maxJerk  = (maxJerk  > 0) ? maxJerk  : 1.0f;
      }

      // call when you want a new move
      // currentPos: where you are now (units)
      void setTarget(float targetPos, float currentPos) {
          _targetPos = targetPos;
          planProfile(currentPos);
          // reset PID terms
          _prevError = 0.0f;
          _iTerm     = 0.0f;
      }

      // call every loop
      // currentPos: sensor position (same units)
      // dt: seconds
      // returns motor command in [-1, 1]
      float update(float currentPos, float dt) {
          if (dt <= 0.0f) dt = 1e-3f;

          advancePlanned(dt);  // advance along solved 7-seg profile

          // PID to follow the planned profile
          float posError = _planPos - currentPos;

          _iTerm += posError * dt;
          if (_iTerm > _iLimit)  _iTerm = _iLimit;
          if (_iTerm < -_iLimit) _iTerm = -_iLimit;

          float dTerm = (posError - _prevError) / dt;
          _prevError  = posError;

          // velocity feedforward
          float ff = 0.0f;
          if (_maxVel > 1e-6f)
              ff = _kF * (_planVel / _maxVel);

          float out = ff
                      + _kP * posError
                      + _kI * _iTerm
                      + _kD * dTerm;

          if (out > 1.0f) out = 1.0f;
          if (out < -1.0f) out = -1.0f;
          return out;
      }

      // optional: where the profile thinks it is
      float plannedPosition() const { return _planPos; }
      float plannedVelocity() const { return _planVel; }

  private:
      struct Seg {
          float dt;
          float j;   // jerk for this segment (signed, already has direction)
      };

      // trajectory data
      Seg   _seg[7];
      float _totalTime = 0.0f;
      float _elapsed   = 0.0f;
      int   _curSeg    = 0;

      float _targetPos = 0.0f;
      float _direction = 1.0f;

      // current planned state
      float _planPos = 0.0f;
      float _planVel = 0.0f;
      float _planAcc = 0.0f;

      // constraints
      float _maxVel   = 1000.0f;
      float _maxAccel = 2000.0f;
      float _maxJerk  = 4000.0f;

      // PID
      float _kP = 0.0f;
      float _kI = 0.0f;
      float _kD = 0.0f;
      float _kF = 0.0f;
      float _prevError = 0.0f;
      float _iTerm     = 0.0f;
      float _iLimit    = 0.2f;

      // -----------------------
      // profile planning
      // -----------------------
      void planProfile(float currentPos) {
          float D = _targetPos - currentPos;
          _direction = (D >= 0.0f) ? 1.0f : -1.0f;
          float Dabs = (D >= 0.0f) ? D : -D;

          // rename for brevity
          float V = _maxVel;
          float A = _maxAccel;
          float J = _maxJerk;

          // minimal distance with triangular S-curve (no const accel, no cruise)
          // Dmin = 2 * A^3 / J^2
          float Dmin = 2.0f * (A*A*A) / (J*J);

          float t1, t2, t3, t4, t5, t6, t7;
          t1=t2=t3=t4=t5=t6=t7=0.0f;

          if (Dabs < 1e-6f) {
              // no move
              for (int i=0;i<7;i++){ _seg[i].dt=0; _seg[i].j=0; }
              _planPos = _targetPos;
              _planVel = 0.0f;
              _planAcc = 0.0f;
              _elapsed = 0.0f;
              _curSeg  = 0;
              _totalTime = 0.0f;
              return;
          }

          if (Dabs < Dmin) {
              // tiny move: reduce peak accel to fit distance, triangular S-curve
              // Dabs = 2 * A'^3 / J^2  ->  A' = cbrt( Dabs * J^2 / 2 )
              float Apeak = pow(Dabs * J*J / 2.0f, 1.0f/3.0f);
              float tJ = Apeak / J;

              t1 = tJ; t2 = 0; t3 = tJ;
              t4 = 0;
              t5 = tJ; t6 = 0; t7 = tJ;

              // fill segments with direction
              _seg[0] = { t1,  _direction *  J };
              _seg[1] = { t2,  _direction *  0 };
              _seg[2] = { t3,  _direction * -J };
              _seg[3] = { t4,  _direction *  0 };
              _seg[4] = { t5,  _direction * -J };
              _seg[5] = { t6,  _direction *  0 };
              _seg[6] = { t7,  _direction *  J };
          } else {
              // normal or cruise case, with full A and J
              float tJ = A / J;

              // velocity reached with t2=0
              float v_no_t2 = (A*A) / J;  // v after seg3 if t2=0

              // make sure V is feasible; if user sets V < v_no_t2, we still run but note it's a limit
              // we will solve two cases:
              // 1) D supports cruise at V
              // 2) D does not, so t4 = 0 and we solve for t2

              // first compute accel distance/time to reach V (with t2 >= 0)
              float t2_for_V = (V - v_no_t2) / A;  // could be < 0
              if (t2_for_V < 0.0f) {
                  // user asked for too low V; just clamp to t2=0 and use v_no_t2 as cruise vel
                  t2_for_V = 0.0f;
                  V = v_no_t2;
              }

              // distance accelerated when we do reach V
              // sAcc(t2) = A^3/J^2 + 1.5*(A^2/J)*t2 + 0.5*A*t2^2
              float sAcc_at_V =
                  (A*A*A)/(J*J)
                  + 1.5f*(A*A/J)*t2_for_V
                  + 0.5f*A*t2_for_V*t2_for_V;

              float sNoCruise_at_V = 2.0f * sAcc_at_V;

              if (Dabs > sNoCruise_at_V) {
                  // Case 1: we can cruise at V
                  float sCruise = Dabs - sNoCruise_at_V;
                  float tCruise = sCruise / V;

                  t1 = tJ;
                  t2 = t2_for_V;
                  t3 = tJ;
                  t4 = tCruise;
                  t5 = tJ;
                  t6 = t2_for_V;
                  t7 = tJ;
              } else {
                  // Case 2: no cruise, solve for t2 so that Dabs = 2*sAcc(t2)
                  // Dabs/2 = A^3/J^2 + 1.5*(A^2/J)*t2 + 0.5*A*t2^2
                  // 0.5*A*t2^2 + 1.5*(A^2/J)*t2 + (A^3/J^2 - Dabs/2) = 0
                  float c2 = 0.5f * A;
                  float c1 = 1.5f * (A*A / J);
                  float c0 = (A*A*A) / (J*J) - 0.5f * Dabs;
                  float disc = c1*c1 - 4.0f*c2*c0;
                  if (disc < 0.0f) disc = 0.0f;
                  float t2_sol = (-c1 + sqrt(disc)) / (2.0f*c2);
                  if (t2_sol < 0.0f) t2_sol = 0.0f;

                  t1 = tJ;
                  t2 = t2_sol;
                  t3 = tJ;
                  t4 = 0.0f;
                  t5 = tJ;
                  t6 = t2_sol;
                  t7 = tJ;
              }

              // fill segments
              _seg[0] = { t1,  _direction *  J };
              _seg[1] = { t2,  _direction *  0 };
              _seg[2] = { t3,  _direction * -J };
              _seg[3] = { t4,  _direction *  0 };
              _seg[4] = { t5,  _direction * -J };
              _seg[5] = { t6,  _direction *  0 };
              _seg[6] = { t7,  _direction *  J };
          }

          // init integration state
          _planPos = currentPos;
          _planVel = 0.0f;
          _planAcc = 0.0f;

          _elapsed = 0.0f;
          _curSeg  = 0;

          _totalTime = 0.0f;
          for (int i=0;i<7;i++) _totalTime += _seg[i].dt;
      }

      // integrate along preplanned segments
      void advancePlanned(float dt) {
          if (_curSeg >= 7) {
              // already finished
              _planAcc = 0.0f;
              _planVel = 0.0f;
              _planPos = _targetPos;
              return;
          }

          float remain = dt;
          while (remain > 1e-9f && _curSeg < 7) {
              float segTimeLeft = _seg[_curSeg].dt - (_elapsedInSeg);
              float step = (remain < segTimeLeft) ? remain : segTimeLeft;
              float j = _seg[_curSeg].j;

              // integrate jerk -> accel -> vel -> pos over 'step'
              // assume jerk constant over step
              float newAcc = _planAcc + j * step;
              float avgAcc = (_planAcc + newAcc) * 0.5f;
              float newVel = _planVel + avgAcc * step;
              float avgVel = (_planVel + newVel) * 0.5f;
              float newPos = _planPos + avgVel * step;

              _planAcc = newAcc;
              _planVel = newVel;
              _planPos = newPos;

              _elapsedInSeg += step;
              _elapsed       += step;
              remain         -= step;

              if (_elapsedInSeg >= _seg[_curSeg].dt - 1e-9f) {
                  // move to next segment
                  _curSeg++;
                  _elapsedInSeg = 0.0f;
              }
          }

          if (_curSeg >= 7) {
              // snap to target
              _planAcc = 0.0f;
              _planVel = 0.0f;
              _planPos = _targetPos;
          }
      }

      float _elapsedInSeg = 0.0f;
};

#endif
