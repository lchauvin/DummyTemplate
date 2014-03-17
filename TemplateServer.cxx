#include "TemplateServer.h"

// Template Dimensions
// -- Offset from Z-Frame
#define TEMPLATE_HOLE_OFFSET_FROM_ZFRAME_X  40.0
#define TEMPLATE_HOLE_OFFSET_FROM_ZFRAME_Y  30.3
#define TEMPLATE_HOLE_OFFSET_FROM_ZFRAME_Z  30.0

// -- Corner position
#define TEMPLATE_BLOCK_OFFSET_FROM_HOLE_X   15.0
#define TEMPLATE_BLOCK_OFFSET_FROM_HOLE_Y   15.0
#define TEMPLATE_BLOCK_OFFSET_FROM_HOLE_Z   0.0

// -- Dimensions (mm)
#define TEMPLATE_WIDTH   100.0
#define TEMPLATE_HEIGHT  120.0
#define TEMPLATE_DEPTH   25.0

// -- Number of holes
#define TEMPLATE_NUMBER_OF_GRIDS_X 17
#define TEMPLATE_NUMBER_OF_GRIDS_Y 19

// -- Hole spacing
#define TEMPLATE_GRID_PITCH_X -5
#define TEMPLATE_GRID_PITCH_Y -4.33

// -- Hole size
#define TEMPLATE_HOLE_RADIUS 1.5

// -- Expected Needle length
#define NEEDLE_LENGTH 100


//------------------------------------------------------------
TemplateServer::TemplateServer()
{
  this->StartupStatus     = WAITING;
  this->ZeroingStatus     = WAITING;
  this->CalibrationStatus = WAITING;
  this->TargetingStatus   = WAITING;

  this->ServerSocket = NULL;
  this->Socket       = NULL;

  this->TargetHoleIndex[0] = 0;
  this->TargetHoleIndex[1] = 0;
  this->TargetDepth        = 0.0;
}

//------------------------------------------------------------
TemplateServer::~TemplateServer()
{
}

//------------------------------------------------------------
int TemplateServer::Initialize(int port)
{
  std::cerr << "Initialization...";

  if (port <= 0 || port > 65535)
    {
    std::cerr << "Failed" << std::endl;
    return EXIT_FAILURE;
    }

  this->ServerSocket = igtl::ServerSocket::New();
  int r = this->ServerSocket->CreateServer(port);

  if (r < 0)
    {
    std::cerr << "Failed" << std::endl;
    return EXIT_FAILURE;
    }

  std::cerr << "Succeed" << std::endl;
  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::Run()
{
  if (this->ServerSocket.IsNull())
    {
    std::cerr << "ERROR: Socket not initialized" << std::endl;
    return EXIT_FAILURE;
    }

  std::cerr << "Waiting for client...";

  // Set 10s timeout
  int timeout = 10;

  while(timeout > 0)
    {
    this->Socket = this->ServerSocket->WaitForConnection(1000);

    if (this->Socket.IsNotNull())
      {
      std::cerr << "Connected" << std::endl;

      this->ReceiveMessages();

      return EXIT_SUCCESS;
      }

    timeout--;
    }

  std::cerr << "Timeout" << std::endl;
  return EXIT_FAILURE;
}

//------------------------------------------------------------
int TemplateServer::ReceiveMessages()
{
  igtl::MessageHeader::Pointer headerMsg;
  headerMsg = igtl::MessageHeader::New();

  igtl::TimeStamp::Pointer ts;
  ts = igtl::TimeStamp::New();

  while(1)
    {
    headerMsg->InitPack();

    int r = this->Socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
    if (r == 0)
      {
      this->Socket->CloseSocket();
      }
    if (r != headerMsg->GetPackSize())
      {
      continue;
      }

    headerMsg->Unpack();

    igtlUint32 sec;
    igtlUint32 nanosec;
    headerMsg->GetTimeStamp(ts);
    ts->GetTimeStamp(&sec, &nanosec);

    std::cerr << "Received " << headerMsg->GetDeviceType() << " message..." << std::endl;
    if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0)
      {
      ReceivedTransform(headerMsg);
      }
    else if (strcmp(headerMsg->GetDeviceType(), "STRING") == 0)
      {
      ReceivedString(headerMsg);
      }
    else if (strcmp(headerMsg->GetDeviceType(), "STATUS") == 0)
      {
      ReceivedStatus(headerMsg);
      }
    }
}

// MESSAGE PROCESSING
//------------------------------------------------------------
int TemplateServer::ReceivedTransform(igtl::MessageHeader* header)
{
  std::cerr << "[TRANSFORM]" << std::endl;

  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetMessageHeader(header);
  transMsg->AllocatePack();

  this->Socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());
  int c = transMsg->Unpack(1);
  if (c & igtl::MessageHeader::UNPACK_BODY)
    {
    igtl::Matrix4x4 receivedMatrix;
    transMsg->GetMatrix(receivedMatrix);

    std::cerr << "Received (" << transMsg->GetDeviceName() << "): " << std::endl;
    igtl::PrintMatrix(receivedMatrix);
    std::cerr << std::endl;

    // Send Acknowledgement
    int queryID = atoi(transMsg->GetDeviceName() + 4);

    std::stringstream deviceName;
    deviceName << "ACK_" << std::setw(4) << std::setfill('0') << queryID;

    igtl::TransformMessage::Pointer replyTransform;
    replyTransform = igtl::TransformMessage::New();
    replyTransform->SetDeviceName(deviceName.str().c_str());
    replyTransform->SetMatrix(receivedMatrix);
    replyTransform->Pack();

    std::cerr << "Sending (" << replyTransform->GetDeviceName() << "): " << std::endl;
    igtl::PrintMatrix(receivedMatrix);
    std::cerr << std::endl;

    this->Socket->Send(replyTransform->GetPackPointer(), replyTransform->GetPackSize());

    if (this->StartupStatus == DONE &&
        this->CalibrationStatus == RUNNING &&
        strstr(header->GetDeviceName(), "CLB_") != NULL)
      {
      this->SetRegistrationMatrix(receivedMatrix);
      }
    else if (this->StartupStatus == DONE &&
             this->CalibrationStatus == DONE &&
             this->TargetingStatus == RUNNING &&
             strstr(header->GetDeviceName(), "TGT_") != NULL)
      {
      int status = this->SetTarget(receivedMatrix);
      this->SendStatus(status);
      if (status == EXIT_SUCCESS)
        {
        this->SendSnappedTarget();
        }
      }
    }

  std::cerr << "[/TRANSFORM]" << std::endl << std::endl;
}

//------------------------------------------------------------
int TemplateServer::ReceivedString(igtl::MessageHeader* header)
{
  std::cerr << "[STRING]" << std::endl;

  if (strstr(header->GetDeviceName(), "CMD_") != NULL)
    {
    igtl::StringMessage::Pointer stringMsg;
    stringMsg = igtl::StringMessage::New();
    stringMsg->SetMessageHeader(header);
    stringMsg->AllocatePack();

    this->Socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize());

    int c = stringMsg->Unpack(1);
    if (c & igtl::MessageHeader::UNPACK_BODY)
      {
      // Get String
      std::string receivedString(stringMsg->GetString());
      std::cerr << "Received (" << stringMsg->GetDeviceName() << "): " << receivedString << std::endl;

      // Send Acknowledgement
      int queryID = atoi(stringMsg->GetDeviceName() + 4);

      std::stringstream deviceName;
      deviceName << "ACK_" << std::setw(4) << std::setfill('0') << queryID;

      igtl::StringMessage::Pointer replyString;
      replyString = igtl::StringMessage::New();
      replyString->SetDeviceName(deviceName.str().c_str());
      replyString->SetString(receivedString);
      replyString->Pack();

      std::cerr << "Sending (" << replyString->GetDeviceName() << "): " << replyString->GetString() << std::endl;
      this->Socket->Send(replyString->GetPackPointer(), replyString->GetPackSize());

      // Execute state
      if (receivedString.compare("START_UP") == 0)
        {
        this->Startup();
        }
      else if (receivedString.compare("CALIBRATION") == 0)
        {
        this->Calibration();
        }
      else if (receivedString.compare("TARGETING") == 0)
        {
        this->Targeting();
        }

      std::cerr << "[/STRING]" << std::endl << std::endl;
      return EXIT_SUCCESS;
      }

    std::cerr << "ERROR: Cannot unpack message or wrong checksum" << std::endl;
    std::cerr << "[/STRING]" << std::endl << std::endl;
    return EXIT_FAILURE;
    }
}

//------------------------------------------------------------
int TemplateServer::ReceivedStatus(igtl::MessageHeader* header)
{
  std::cerr << "[STATUS]" << std::endl;

  std::cerr << "[/STATUS]" << std::endl << std::endl;
  return EXIT_SUCCESS;
}

// STATE EXECUTION
//------------------------------------------------------------
int TemplateServer::Startup()
{
  std::cerr << "  [STARTUP]" << std::endl;
  this->StartupStatus = RUNNING;

  if (this->SendZFrameConfiguration() == EXIT_FAILURE)
    {
    this->StartupStatus = FAILED;
    return EXIT_FAILURE;
    }

  this->StartupStatus = DONE;
  std::cerr << "  [/STARTUP]" << std::endl;

  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::SendZFrameConfiguration()
{
  if (this->Socket.IsNull())
    {
    return EXIT_FAILURE;
    }

  std::cerr << "  Sending ZFrame Configuration...";

  igtl::TrajectoryMessage::Pointer trajMsg;
  trajMsg = igtl::TrajectoryMessage::New();
  trajMsg->SetDeviceName("ZFrameConfig");

  igtl::TrajectoryElement::Pointer line0;
  line0 = igtl::TrajectoryElement::New();
  line0->SetType(igtl::TrajectoryElement::TYPE_ENTRY_TARGET);
  line0->SetEntryPosition(30.0, -30.0, -30.0);
  line0->SetTargetPosition(30.0, -30.0, 30.0);
  trajMsg->AddTrajectoryElement(line0);

  igtl::TrajectoryElement::Pointer line1;
  line1 = igtl::TrajectoryElement::New();
  line1->SetType(igtl::TrajectoryElement::TYPE_ENTRY_TARGET);
  line1->SetEntryPosition(30.0, -30.0, 30.0);
  line1->SetTargetPosition(30.0, 30.0, -30.0);
  trajMsg->AddTrajectoryElement(line1);

  igtl::TrajectoryElement::Pointer line2;
  line2 = igtl::TrajectoryElement::New();
  line2->SetType(igtl::TrajectoryElement::TYPE_ENTRY_TARGET);
  line2->SetEntryPosition(30.0, 30.0, -30.0);
  line2->SetTargetPosition(30.0, 30.0, 30.0);
  trajMsg->AddTrajectoryElement(line2);

  igtl::TrajectoryElement::Pointer line3;
  line3 = igtl::TrajectoryElement::New();
  line3->SetType(igtl::TrajectoryElement::TYPE_ENTRY_TARGET);
  line3->SetEntryPosition(30.0, 30.0, 30.0);
  line3->SetTargetPosition(-30.0, 30.0, -30.0);
  trajMsg->AddTrajectoryElement(line3);

  igtl::TrajectoryElement::Pointer line4;
  line4 = igtl::TrajectoryElement::New();
  line4->SetType(igtl::TrajectoryElement::TYPE_ENTRY_TARGET);
  line4->SetEntryPosition(-30.0, 30.0, -30.0);
  line4->SetTargetPosition(-30.0, 30.0, 30.0);
  trajMsg->AddTrajectoryElement(line4);

  igtl::TrajectoryElement::Pointer line5;
  line5 = igtl::TrajectoryElement::New();
  line5->SetType(igtl::TrajectoryElement::TYPE_ENTRY_TARGET);
  line5->SetEntryPosition(-30.0, 30.0, 30.0);
  line5->SetTargetPosition(-30.0, -30.0, -30.0);
  trajMsg->AddTrajectoryElement(line5);

  igtl::TrajectoryElement::Pointer line6;
  line6 = igtl::TrajectoryElement::New();
  line6->SetType(igtl::TrajectoryElement::TYPE_ENTRY_TARGET);
  line6->SetEntryPosition(-30.0, -30.0, -30.0);
  line6->SetTargetPosition(-30.0, -30.0, 30.0);
  trajMsg->AddTrajectoryElement(line6);

  trajMsg->Pack();
  this->Socket->Send(trajMsg->GetPackPointer(), trajMsg->GetPackSize());

  std::cerr << "Done" << std::endl;

  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::Calibration()
{
  if (this->StartupStatus == DONE)
    {
    this->CalibrationStatus = RUNNING;
    return EXIT_SUCCESS;
    }
  return EXIT_FAILURE;
}

//------------------------------------------------------------
int TemplateServer::Targeting()
{
  if (this->StartupStatus == DONE &&
      this->CalibrationStatus == DONE)
    {
    this->TargetingStatus = RUNNING;
    return EXIT_SUCCESS;
    }
  return EXIT_FAILURE;
}

//------------------------------------------------------------
int TemplateServer::SetRegistrationMatrix(igtl::Matrix4x4 &matrix)
{
  std::cerr << "  [CALIBRATION]" << std::endl;

  for (int i = 0; i < 4; ++i)
    {
    for (int j = 0; j < 4; ++j)
      {
      this->RegistrationMatrix[i][j] = matrix[i][j];
      }
    }
  this->CalibrationStatus = DONE;

  std::cerr << "  [/CALIBRATION]" << std::endl;
  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::SetTarget(igtl::Matrix4x4 &matrix)
{
  std::cerr << "  [TARGET]" << std::endl;

  for (int i = 0; i < 4; ++i)
    {
    for (int j = 0; j < 4; ++j)
      {
      this->TargetReceived[i][j] = matrix[i][j];
      }
    }

  if ( this->Kinematics() == EXIT_FAILURE )
    {
    return EXIT_FAILURE;
    }
  
  std::cerr << "  [/TARGET]" << std::endl;
  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::Kinematics()
{
  double target[3] = { this->TargetReceived[0][3],
                       this->TargetReceived[1][3],
                       this->TargetReceived[2][3] };

  if ( this->FindHoleIndex(target, this->TargetHoleIndex, this->TargetDepth) == EXIT_FAILURE )
    {
    return EXIT_FAILURE;
    }

  this->GetHoleTransform(this->TargetHoleIndex[0], this->TargetHoleIndex[1], this->TargetSnapped);
  
  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::FindHoleIndex(double* target, int* index, double& depth)
{
  index[0] = 0;
  index[1] = 0;
  igtl::Matrix4x4 temp;

  if ( this->GetHoleTransform(index[0], index[1], temp) == EXIT_FAILURE )
    {
    return EXIT_FAILURE;
    }
  
  double holeX = temp[0][3];
  double holeY = temp[1][3];
  double holeZ = temp[2][3];

  depth = std::sqrt((holeX-target[0])*(holeX-target[0])+(holeY-target[1])*(holeY-target[1])+(holeZ-target[2])*(holeZ-target[2]));

  for (int i = 0; i < TEMPLATE_NUMBER_OF_GRIDS_Y; ++i)
    {
    int nOfHoles = TEMPLATE_NUMBER_OF_GRIDS_X;
    if (i % 2 == 0)
      {
      nOfHoles--;
      }

    for (int j = 0; j < nOfHoles; ++j)
      {
      this->GetHoleTransform(i,j,temp);
      holeX = temp[0][3];
      holeY = temp[1][3];
      holeZ = temp[2][3];

      double dist = std::sqrt((holeX-target[0])*(holeX-target[0])+(holeY-target[1])*(holeY-target[1])+(holeZ-target[2])*(holeZ-target[2]));

      if (dist < depth)
	{
	depth = dist;
	index[0] = i;
	index[1] = j;
	}
      }
    }

  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::GetHoleTransform(int i, int j, igtl::Matrix4x4& matrix)
{
  int nOfHoles = TEMPLATE_NUMBER_OF_GRIDS_X;
  if (i % 2 == 0)
    {
    nOfHoles--;
    }

  if (i < 0 || i >= TEMPLATE_NUMBER_OF_GRIDS_Y ||
      j < 0 || j >= nOfHoles)
    {
    // the grid index is out of range
    std::cerr << "(" << i << "," << j << ")" << std::endl;
    std::cerr << "  Out-of-range" << std::endl;
    return EXIT_FAILURE;
    }

  double shiftingLineOffset = 0.0;
  if (i % 2 == 0)
    {
    shiftingLineOffset = TEMPLATE_GRID_PITCH_X/2;
    }

  // offset from the Z-frame center
  double off[4];
  off[0] = TEMPLATE_HOLE_OFFSET_FROM_ZFRAME_X + shiftingLineOffset + j * TEMPLATE_GRID_PITCH_X;
  off[1] = TEMPLATE_HOLE_OFFSET_FROM_ZFRAME_Y + i * TEMPLATE_GRID_PITCH_Y;
  off[2] = TEMPLATE_HOLE_OFFSET_FROM_ZFRAME_Z;
  off[3] = 1.0;

  double holePosition[3];
  holePosition[0] = this->RegistrationMatrix[0][0]*off[0] + this->RegistrationMatrix[0][1]*off[1] +
    this->RegistrationMatrix[0][2]*off[2] + this->RegistrationMatrix[0][3]*off[3];
  holePosition[1] = this->RegistrationMatrix[1][0]*off[0] + this->RegistrationMatrix[1][1]*off[1] +
    this->RegistrationMatrix[1][2]*off[2] + this->RegistrationMatrix[1][3]*off[3];
  holePosition[2] = this->RegistrationMatrix[2][0]*off[0] + this->RegistrationMatrix[2][1]*off[1] +
    this->RegistrationMatrix[2][2]*off[2] + this->RegistrationMatrix[2][3]*off[3];

  for (int i=0; i<4; ++i)
    {
    for (int j=0; j<4; ++j)
      {
      matrix[i][j] = this->RegistrationMatrix[i][j];
      }
    }
  matrix[0][3] = holePosition[0];
  matrix[1][3] = holePosition[1];
  matrix[2][3] = holePosition[2];

  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::SendStatus(int status)
{
  std::cerr << "  [STATUS]" << std::endl;

  int j_corrected = this->TargetHoleIndex[1] - 
    ((this->TargetHoleIndex[0]%2 == 0 && this->TargetHoleIndex[1]>7)? 7: 8);
  std::stringstream index;
  index << (char)('A'+this->TargetHoleIndex[0]) << "," << j_corrected  << "," << this->TargetDepth;

  igtl::StatusMessage::Pointer statusMessage;
  statusMessage = igtl::StatusMessage::New();
  statusMessage->SetDeviceName("TARGET");
  if (status == EXIT_SUCCESS)
    {
    statusMessage->SetCode(igtl::StatusMessage::STATUS_OK); 
    statusMessage->SetStatusString(index.str().c_str());
    }
  else
    {
    // Out-of-range
    statusMessage->SetCode(igtl::StatusMessage::STATUS_CONFIG_ERROR); 
    }
  statusMessage->Pack();
  
  this->Socket->Send(statusMessage->GetPackPointer(), statusMessage->GetPackSize());

  std::cerr << "  Index,Depth:" << index.str() << std::endl;
  std::cerr << "  [/STATUS]" << std::endl;

  return EXIT_SUCCESS;
}

//------------------------------------------------------------
int TemplateServer::SendSnappedTarget()
{
  std::cerr << "  [SNAPPED]" << std::endl;

  igtl::TransformMessage::Pointer snappedTransform;
  snappedTransform = igtl::TransformMessage::New();
  snappedTransform->SetDeviceName("TARGET");
  snappedTransform->SetMatrix(this->TargetSnapped);
  snappedTransform->Pack();

  this->Socket->Send(snappedTransform->GetPackPointer(), snappedTransform->GetPackSize());
  
  igtl::PrintMatrix(this->TargetSnapped);

  std::cerr << "  [/SNAPPED]" << std::endl;

  return EXIT_SUCCESS;
}
