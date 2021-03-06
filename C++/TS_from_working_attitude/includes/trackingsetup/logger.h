/*
 * TAerrorLogger.h
 *
 *  Created on: Jun 15, 2013
 *      Author: thomas
 */

#ifndef TALOGGER_H_
#define TALOGGER_H_

#include <sstream>

#include "trackingsetup/trackingsetup.h"
#include "trackingsetup/types.h"


namespace tracking {

class Logger {
public:
	Logger(std::string _logFileName, verbosityLevel _verbosity,
			bool _noStdout);

	void log(verbosityLevel _vLvl, std::string message);

	void log(time_t timestamp, verbosityLevel vLvl, std::string message);

	void add(std::list<TALogMessage>*);

	std::string verbosityLevelName(verbosityLevel _vLvl);

	void registerInstance(TrackingSetup* instance);

	/*
	 * get all logs from the instances registered with @registerLoggingInstance(TAclass*)
	 */
	void fetchLogs();

	void setCacheLogForGui(bool val);

	void clearCacheForGui();

	friend std::ostream& operator<<(std::ostream& out, const Logger& log);

private:
	verbosityLevel verbosity;
	std::fstream logFileStream;
	bool useStdout;

	// create a list with all classes writing to log
	std::list<TrackingSetup*> loggingInstances;

	//
	bool cacheLogForGui;

	// stringstream acting as buffer for gui backend
	std::ostringstream GuiStreambuffer;

	// need to synchronize access to GuiStreambuffer
	pthread_mutex_t* pGuiBufferMutex;
};

} /* namespace tracking */
#endif /* TALOGGER_H_ */
