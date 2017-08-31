#ifndef COSTMAP_ARRAY_PARSER_H_MWH_20170831
#define COSTMAP_ARRAY_PARSER_H_MWH_20170831

#include <vector>
#include <string>

namespace costmap
{

std::vector<std::vector<float> > parseVVF(const std::string& input, std::string& error_return);

}

#endif