#include <functional>
#include <cstdlib>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <random>
#include <cmath>
#include <utility>

const int BeaconTag = 0;
const int Robot1Tag = 0;
const int Robot2Tag = 0;
const double BeaconNoise = 0.00005;
const double robot1Noise = 0.0005;
const double robot2Noise = 0.0005;
const double BeaconProcessnoise = 0.00005;
const double robot1processNoise = 0.0005;
const double robot2processNoise = 0.0005;
const double C1 = 0.60;
const double C2 = 0.60;


namespace gazebo
{
  class Beacon : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->world = _parent;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Beacon::OnUpdate, this));
        this->robot1 = this -> world -> ModelByName("Robot1");
        this->robot2 = this -> world -> ModelByName("Robot1_0");
        this->name = this -> robot1 -> GetName();
        this->name1 = this -> robot2 -> GetName();
        this->pose1 = this -> robot1 -> WorldPose();
        this->pose2 = this -> robot2 -> WorldPose();
        this->lastposR1 = this -> pose1.Pos();
        this->lastposR2 = this -> pose2.Pos();
        this -> sensedMoveR1 = this -> applyNoise(0,lastposR1.X(),lastposR1.Y(),lastposR1.Z(),this -> pose1.Pos().X(),this -> pose1.Pos().Y(), this -> pose1.Pos().Z());
        this -> sensedMoveR2 = this -> applyNoise(0,lastposR2.X(),lastposR2.Y(),lastposR2.Z(),this -> pose2.Pos().X(),this -> pose2.Pos().Y(), this -> pose2.Pos().Z());
        this->iterX1[0] = pose1.Pos().X();
        this->iterX1[1] = pose1.Pos().Y();
        this->iterX1[2] = sensedMoveR1.X();
        this->iterX1[3] = sensedMoveR1.Y();

        this->iterX2[0] = pose2.Pos().X();
        this->iterX2[1] = pose2.Pos().Y();
        this->iterX2[2] = sensedMoveR2.X();
        this->iterX2[3] = sensedMoveR2.Y();

        ignition::math::Vector3<double> difftemp = applyNoise(0,pose2.Pos().X(),pose2.Pos().Y(),pose2.Pos().Z(),pose1.Pos().X(),pose1.Pos().Y(),pose1.Pos().Z());

        this->iterBec[0] = difftemp.X();
        this->iterBec[1] = difftemp.Y();
        this->iterBec[2] = difftemp.X();
        this->iterBec[3] = difftemp.Y();
    }

    public: void OnUpdate()
    {
      pose1 = this -> robot1 -> WorldPose();
      pose2 = this -> robot2 -> WorldPose();
      ignition::math::Vector3<double> diff = applyNoise(BeaconNoise,pose2.Pos().X(),pose2.Pos().Y(),pose2.Pos().Z(),pose1.Pos().X(),pose1.Pos().Y(),pose1.Pos().Z());
      ignition::math::Vector3<double> realdif = applyNoise(0,pose2.Pos().X(),pose2.Pos().Y(),pose2.Pos().Z(),pose1.Pos().X(),pose1.Pos().Y(),pose1.Pos().Z());
      if(lastposR1.X() != pose1.Pos().X() || lastposR1.Y() != pose1.Pos().Y()){
        this -> sensedMoveR1 = applyNoise(robot1Noise,lastposR1.X(),lastposR1.Y(),lastposR1.Z(),pose1.Pos().X(),pose1.Pos().Y(),pose1.Pos().Z());
        this -> lastposR1 = pose1.Pos();
      }

      if(lastposR2.X() != pose2.Pos().X() || lastposR2.Y() != pose2.Pos().Y()){
        this -> sensedMoveR2 = applyNoise(robot2Noise,lastposR2.X(),lastposR2.Y(),lastposR2.Z(),pose2.Pos().X(), pose2.Pos().Y(),pose2.Pos().Z());
        this -> lastposR2 = pose2.Pos();
      }

      this -> iterX1[0] = sensedMoveR1.X() + iterX1[0];
      this -> iterX1[1] = sensedMoveR1.Y() + iterX1[1];
      this -> iterX2[0] = sensedMoveR2.X() + iterX2[0];
      this -> iterX2[1] = sensedMoveR2.Y() + iterX2[1];
      double M1[2] = {iterX1[0],iterX1[1]};
      double M2[2] = {iterX2[0],iterX2[1]};
      double MB[2] = {diff.X(),diff.Y()};

      kalmanFilter(this -> iterX1,this -> iterP1,M1,robot1processNoise,Robot1Tag);
      kalmanFilter(this -> iterX2,this -> iterP2,M2,robot2processNoise,Robot2Tag);
      kalmanFilter(this -> iterBec,this -> iterPBec,MB,BeaconProcessnoise,BeaconTag);

      //Apply technique here, newX1 =  (1-c)x1' + c(beacon + x1'), newX2 =  (1-c)x2' + c(beacon + x2')
      this -> iterX1[0] = (1-C1)*(this -> iterX1[0]) + C1*((this -> iterX2[0]) + diff.X());
      this -> iterX1[1] = (1-C1)*(this -> iterX1[1]) + C1*((this -> iterX2[1]) + diff.Y());

      this -> iterX2[0] = (1-C2)*(this -> iterX2[0]) + C2*((this -> iterX1[0]) - diff.X());
      this -> iterX2[1] = (1-C2)*(this -> iterX2[1]) + C2*((this -> iterX1[1]) - diff.Y());
      this -> count = this -> count + 1;

      if(this -> count >= 1000){
        printf("%s: %d\n","iteration", iteration);
        printf("%s: %f %f\n",this -> name.c_str(),pose1.Pos().X(),pose1.Pos().Y());
        printf("%s: %f %f\n",this -> name1.c_str(),pose2.Pos().X(),pose2.Pos().Y());
        printf("%s: %f %f\n","real dif", realdif.X(),realdif.Y());
        printf("%s: %f %f\n","iterx1 R1",iterX1[0],iterX1[1]);
        printf("%s: %f %f\n","iterx2 R2",iterX2[0],iterX2[1]);
        printf("%s: %f %f\n","iterBeacon RB",iterBec[0],iterBec[1]);
        double sR1x = 0;
        double sR1y = 0;
        double sR2x = 0;
        double sR2y = 0;
        for(int i = 0; i < 1000; i = i + 1){
          sR1x = sR1x + this -> sumErrorR1[0][i];
          sR1y = sR1y + this -> sumErrorR1[1][i];
          sR2x = sR2x + this -> sumErrorR2[0][i];
          sR2y = sR2y + this -> sumErrorR2[1][i];
        }
        printf("%s: %f %f\n","error rate Robot 1", sR1x/1000, sR1y/1000);
        printf("%s: %f %f\n\n","error rate Robot 2", sR2x/1000, sR2y/1000);                
        this -> count = 0;
        this -> iteration = iteration + 1;
      }
      else{
        this -> sumErrorR1[0][count] = std::abs((iterX1[0] - pose1.Pos().X())/pose1.Pos().X());
        this -> sumErrorR1[1][count] = std::abs((iterX1[1] - pose1.Pos().Y())/pose1.Pos().Y());
        this -> sumErrorR2[0][count] = std::abs((iterX2[0] - pose2.Pos().X())/pose2.Pos().X());
        this -> sumErrorR2[1][count] = std::abs((iterX2[1] - pose2.Pos().Y())/pose2.Pos().Y());
      }
    }

    public: void kalmanFilter(double X[4], double P[][4],double M[2],double R, int tag)
    {
      double motion[4][1] = {{0},{0},{0},{0}};
      double Q[4][4] = {
        {0.0001,0,0,0},
        {0,0.0001,0,0},
        {0,0,0.0001,0},
        {0,0,0,0.0001}
      };

      double F[4][4] = {
        {1,0,1,0},
        {0,1,0,1},
        {0,0,1,0},
        {0,0,0,1}
      };

      if(tag == 0){
        F[0][0] = 0;
        F[1][1] = 0;
      };

      double FT[4][4] = {
        {1,0,0,0},
        {0,1,0,0},
        {1,0,1,0},
        {0,1,0,1}
      };

      if(tag == 0){
        FT[0][0] = 0;
        FT[1][1] = 0;
      }

      double H[2][4] = {
        {1,0,0,0},
        {0,1,0,0}
      };

      double HT[4][2] = {
        {0,1},
        {1,0},
        {0,0},
        {0,0}
      };

      double I[4][4] = {
        {1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}
      };

      double y[2];
      double htx[2] ={ H[0][0] * X[0],H[1][1] * X[1]}; 
      y[0] = M[0] - htx[0];
      y[1] = M[1] - htx[1];
      double htp[2][4];
      for(int a = 0; a < 2; a = a + 1){
        for(int b = 0; b < 4; b = b + 1){
          double sum = 0;
          for(int c = 0; c <4; c = c + 1){
            sum = sum + H[a][c] * P[c][b]; 
          }
          htp[a][b] = sum;
        }
      }

      double S[2][2];
      for(int a = 0; a < 2; a = a + 1){
        for(int b = 0; b < 2; b = b + 1){
          double sum = 0;
          for(int c = 0; c <4; c = c + 1){
            sum = sum + htp[a][c] * HT[c][b]; 
          }
          S[a][b] = sum + R;
        }
      }

      double K[4][2];
      double ptHT[4][2];
      for(int a = 0; a < 4; a = a + 1){
        for(int b = 0; b < 2; b = b + 1){
          double sum = 0;
          for(int c = 0; c <4; c = c + 1){
            sum = sum + P[a][c] * HT[c][b]; 
          }
          ptHT[a][b] = sum;
        }
      }
      double SI[2][2];
      double deter = (S[0][0] * S[1][1] - S[0][1] * S[1][0]);
      SI[0][0] = S[1][1] / deter;
      SI[0][1] = -S[0][1]/deter;
      SI[1][0] = -S[1][0]/deter;
      SI[1][1] = S[0][0]/deter;

      for(int a = 0; a < 4; a = a + 1){
        for(int b = 0; b < 2; b = b + 1){
          double sum = 0;
          for(int c = 0; c <2; c = c + 1){
            sum = sum + ptHT[a][c] * SI[c][b]; 
          }
          K[a][b] = sum;
        }
      }

      double x[4];
      x[0] = X[0] + (K[0][0] * y[0] + K[0][1] * y[1]); 
      x[1] = X[1] + (K[1][0] * y[0] + K[1][1] * y[1]);
      x[2] = X[2] + (K[2][0] * y[0] + K[2][1] * y[1]);
      x[3] = X[3] + (K[3][0] * y[0] + K[3][1] * y[1]);

      double kh[4][4];
      for(int a = 0; a < 4; a = a + 1){
        for(int b = 0; b < 4; b = b + 1){
          double sum = 0;
          for(int c = 0; c <2; c = c + 1){
            sum = sum + K[a][c] * H[c][b]; 
          }
          K[a][b] = sum;
        }
      }

      double Imkh[4][4];

      for(int a = 0; a < 4; a = a + 1){
        for(int b = 0; b < 4; b = b + 1){
          Imkh[a][b] = I[a][b] - kh[a][b];
        }
      }

      double P1[4][4];

      for(int a = 0; a < 4; a = a + 1){
        for(int b = 0; b < 4; b = b + 1){
          double sum = 0;
          for(int c = 0; c <4; c = c + 1){
            sum = sum + Imkh[a][c] * P[c][b]; 
          }
          P1[a][b] = sum;
        }
      }

      double newx[4];
      newx[0] = F[0][0]*x[0] + F[0][2]*x[2];
      newx[1] = F[1][1]*x[1] + F[1][3]*x[3];
      newx[2] = F[2][2]*x[2];
      newx[3] = F[3][3]*x[3]; 

      double FtP[4][4];
      for(int a = 0; a < 4; a = a + 1){
        for(int b = 0; b < 4; b = b + 1){
          double sum = 0;
          for(int c = 0; c <4; c = c + 1){
            sum = sum + F[a][c] * P[c][b]; 
          }
          FtP[a][b] = sum;
        }
      }
      double newP[4][4];
      for(int a = 0; a < 4; a = a + 1){
        for(int b = 0; b < 4; b = b + 1){
          double sum = 0;
          for(int c = 0; c <4; c = c + 1){
            sum = sum + FtP[a][c] * FT[c][b]; 
          }
          newP[a][b] = sum + Q[a][b];
        }
      }

      if(tag == 1){
        for(int a = 0; a < 4; a = a + 1){
          for(int b = 0; b < 4; b = b + 1){
            this -> iterP1[a][b] = newP[a][b];
          }
        }

        for(int a = 0; a < 4; a = a + 1){
            this -> iterX1[a] = newx[a];
        }
      }
      else if(tag == 2){
        for(int a = 0; a < 4; a = a + 1){
          for(int b = 0; b < 4; b = b + 1){
            this -> iterP2[a][b] = newP[a][b];
          }
        }

        for(int a = 0; a < 4; a = a + 1){
            this -> iterX2[a] = newx[a];
        }
      }
      else{
        for(int a = 0; a < 4; a = a + 1){
          for(int b = 0; b < 4; b = b + 1){
            this -> iterPBec[a][b] = newP[a][b];
          }
        }

        for(int a = 0; a < 4; a = a + 1){
            this -> iterBec[a] = newx[a];
        }
      }
    }

    public: ignition::math::Vector3<double> applyNoise(double noiseLevel,double x1, double y1,double z1,double x2,double y2, double z2)
    {
      const double mean = 0.0;
      const double std = noiseLevel;
      std::normal_distribution<double> dist(mean, std);
      ignition::math::Vector3<double> diff;
      diff.X() = x2 - x1;
      diff.Y() = y2 - y1;
      diff.Z() = z2 - z1;

      diff.X() = diff.X() + dist(generator);
      diff.Y() = diff.Y() + dist(generator);
      diff.Z() = diff.Z() + dist(generator);
      return diff;
    }

    // Pointer to the model
    private: physics::WorldPtr world;
    private: physics::ModelPtr robot1;
    private: physics::ModelPtr robot2;
    private: std::string name;
    private: std::string name1;
    private: ignition::math::Vector3<double> lastposR1;
    private: ignition::math::Vector3<double> lastposR2;
    private: ignition::math::Vector3<double> sensedMoveR1;
    private: ignition::math::Vector3<double> sensedMoveR2;
    private: ignition::math::Pose3d pose1;
    private: ignition::math::Pose3d pose2;
    private: std::default_random_engine generator;
    private: double iterX1[4] = {0,0,0,0};
    private: double iterP1[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    private: double iterX2[4] = {0,0,0,0};
    private: double iterP2[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    private: double iterBec[4] = {0,0,0,0};
    private: double iterPBec[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    private: int count = 0;
    private: int iteration = 0;
    private: double sumErrorR1[2][1000];
    private: double sumErrorR2[2][1000];


    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

// Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Beacon)
}