#include <cstdio>
#include <cstdlib>

#include "TemplateServer.h"

//------------------------------------------------------------
int main(int argc, char** argv)
{
  if (argc < 2)
    {
    std::cerr << "Usage: ./DummyTemplate port" << std::endl
	      << "   port: Server port (default: 18944)" << std::endl;
    return EXIT_FAILURE;
    }

  int port = atoi(argv[1]);

  TemplateServer* tServer = new TemplateServer();
  if (tServer)
    {
    int r = tServer->Initialize(port);
    if (r == EXIT_FAILURE)
      {
      exit(EXIT_FAILURE);
      }

    tServer->Run();
    }

}
