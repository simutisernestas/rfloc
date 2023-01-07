// Modified example from: https://github.com/simondlevy/TinyEKF

#define Nsta 6
#define Mobs 4

#include <TinyEKF.h>

double BEACONS[4][3] = {
  {1, 2, 3},
  {1, 2, 3},
  {1, 2, 3},
  {1, 2, 3},
};

class Fuser : public TinyEKF
{

public:
  Fuser()
  {
    // We approximate the process noise using a small constant
    this->setQ(0, 0, .0001);
    this->setQ(1, 1, .0001);

    // Same for measurement noise
    this->setR(0, 0, .0001);
    this->setR(1, 1, .0001);
    this->setR(2, 2, .0001);
  }

protected:
  void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
  {
    double dt = getdt();

    // Constant velocity model
    fx[0] = this->x[0] + this->x[3] * dt;
    fx[1] = this->x[1] + this->x[4] * dt;
    fx[2] = this->x[2] + this->x[5] * dt;
    fx[3] = this->x[3];
    fx[4] = this->x[4];
    fx[5] = this->x[5];

    // Model Jacobian, equivalent to model
    F[0][0] = 1;
    F[0][3] = dt;
    F[1][1] = 1;
    F[1][4] = dt;
    F[2][2] = 1;
    F[2][5] = dt;
    F[3][3] = 1;
    F[4][4] = 1;
    F[5][5] = 1;

    // Measurement function (L2 norm between agent & beacons)
    for (size_t i = 0; i < 4; i++) {
      double ssum = 0;
      for (size_t j = 0; j < 3; j++) {
        double diff = this->x[j] - beacons[i][j];
        ssum += diff * diff;
      }
      hx[i] = sqrt(ssum);
    }

    // Jacobian of measurement function
    for (size_t i = 0; i < 4; i++) {
      double ssum = 0;
      for (size_t j = 0; j < 3; j++) {
        double diff = this->x[j] - beacons[i][j];
        H[i][j] = diff / hx[i];
      }
    }
  }

private:
  double beacons[4] = {1, 2, 3, 4};
  double getdt() {return .01;}
};

Fuser ekf;

void setup()
{
  Serial.begin(115200);

  // initialize EKF
  for (size_t i = 0; i < Nsta; i++) {
    ekf.setX(i, 0.0);
  }
}


void loop()
{
  double dist = 1.0;
  double z[4] = {dist, dist, dist, dist};
  ekf.step(z);

  // Report measured and predicte/fused values
  Serial.print(z[0]);
  Serial.print(" ");
  Serial.print(z[1]);
  Serial.print(" ");
  Serial.print(z[2]);
  Serial.print(" ");
  Serial.print(z[4]);
  Serial.print(" ");
  Serial.print(ekf.getX(0));
  Serial.print(" ");
  Serial.print(ekf.getX(1));
  Serial.print(" ");
  Serial.println(ekf.getX(2));
}
