/*
 * Empty C++ Application
 */

#include "Runner.h"
#include "AxiBram.h"
#include "MotorController.h"
#include <memory>
#include <RunConfig.h>

//temp
#include <Map/Map.h>
#include <ScanMatching/IcpOptimization.h>
#include <ScanMatching/ScanMatcher.h>

#ifdef MAIN
int main()
{
	auto l_systemRunner = std::make_unique<Runner>();

	std::thread LidarThread(l_systemRunner->lidarScan());
	std::thread MapGeneratorThread(l_systemRunner->generateMap());
	std::thread GyroscopeThread(l_systemRunner->gyroscopeThread());
	while(1)
	{
		sleep(1);
		//std::thread CommunicationThread(RadioCommunication);
		//CommunicationThread.join();
	}
	GyroscopeThread.join();
	MapGeneratorThread.join();
	LidarThread.join();

	return 0;
}

#endif
