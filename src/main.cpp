#ifdef USE_SIMULATOR
#include "simconnect.h"
#else
#include "mpconnect.h"
#endif //USE_SIMULATOR

int main(int argc, char* argv[])
{
#ifdef USE_SIMULATOR
    return SimMain();
#else
    std::string in_file_name = argv[1];
    return PltConnect(in_file_name);
#endif
}























































































