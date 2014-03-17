#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iomanip>

#include "igtlOSUtil.h"
#include "igtlServerSocket.h"
#include "igtlStatusMessage.h"
#include "igtlStringMessage.h"
#include "igtlTrajectoryMessage.h"
#include "igtlTransformMessage.h"

class TemplateServer
{
 public:
  enum Status
  {
    WAITING,
    RUNNING,
    DONE,
    FAILED
  };
 
  // Constructor / Destructor
  TemplateServer();
  ~TemplateServer();

  // Methods
  int Initialize(int port);
  int Run();

  // Getter / Setter
  Status GetStartupStatus() { return StartupStatus; }
  Status GetZeroingStatus() { return ZeroingStatus; }
  Status GetCalibrationStatus() { return CalibrationStatus; }
  Status GetTargetingStatus() { return TargetingStatus; }

  int   SetRegistrationMatrix(igtl::Matrix4x4 &matrix);
  int   SetTarget(igtl::Matrix4x4 &matrix);

 protected:

  // State Status
  Status StartupStatus;
  Status ZeroingStatus;
  Status CalibrationStatus;
  Status TargetingStatus;

  // Socket
  igtl::ServerSocket::Pointer ServerSocket;
  igtl::Socket::Pointer Socket;

  // Matrix
  igtl::Matrix4x4 RegistrationMatrix;
  igtl::Matrix4x4 TargetReceived;
  igtl::Matrix4x4 TargetSnapped;
  int TargetHoleIndex[2];
  double TargetDepth;

  // Methods
  int ReceiveMessages();
  int ReceivedTransform(igtl::MessageHeader* header);
  int ReceivedString(igtl::MessageHeader* header);
  int ReceivedStatus(igtl::MessageHeader* header);
  int SendZFrameConfiguration();
  int Kinematics();
  int FindHoleIndex(double* target, int* index, double& depth);
  int GetHoleTransform(int i, int j, igtl::Matrix4x4& matrix);
  int SendStatus(int status);
  int SendSnappedTarget();

  // States
  int Startup();
  int Calibration();
  int Targeting();
};

