/*
============================================================================
 Name : weblogger.cpp
 Author :
 Version :
 Description : GMAS C++ project source file

 Usage: './yourprogram 2>&1 | ./weblogger' // send cout & cerr both
        './yourprogram 2> | ./weblogger' // send cerr only
        './yourprogram  | ./weblogger' // send cout only
        './yourprogram  | tee >(./firstprogram) >(./weblogger') // send cout to 2 program
============================================================================
*/

#include "tools/cpp-httplib/httplib.h"
#include <iostream>
#include <regex>

int main(int argc, char **argv)
{
    std::string ipaddr;
    std::string log_str;
    httplib::Server svr;

    if (argc > 2)
    {
        std::cout << "Usage: ./weblogger ipaddress\n"
                  << "Or ./weblogger\n"
                  << "(default ip address is 192.168.1.3)\n";

        return -1;
    }

    argc < 2 ? ipaddr = "192.168.1.3" : ipaddr = argv[1];

    static const std::regex pattern(
        R"(^((25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)$)");

    if (!std::regex_match(ipaddr, pattern))
    {
        std::cout << "please input correct IP address\n";
        return -1;
    }

    svr.Get("/", [&](const httplib::Request &req, httplib::Response &res)
            {

		// seting SSE response header
        res.set_header("Content-Type", "text/event-stream");
        res.set_header("Cache-Control", "no-cache");
        res.set_header("Connection", "keep-alive");
        res.set_header("Access-Control-Allow-Origin", "*");

        res.set_content_provider(
            "text/event-stream",  // content type
            [&](size_t, httplib::DataSink& sink) {

				// sink.write("hello, world\n\n", 14);

               for(;;) {

					// std::cin >> log_str;
					std::getline(std::cin, log_str); 

					sink.write(log_str.c_str(), log_str.size());
					sink.write("\n\n", 2); 

					std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                return true;
            }
        ); });

    svr.listen(ipaddr, 8080);

    return 0;
}
