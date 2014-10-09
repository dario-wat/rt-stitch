#include <string>
#include <ctime>
#include <sstream>

const std::string get_current_time() {
	time_t	now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

const std::string to_string(int x) {
	return static_cast<std::ostringstream*>(&(std::ostringstream() << x))->str();
}