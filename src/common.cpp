#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <iomanip>
#include "common.h"

Image ReadImage(std::string& filename);

StartupOptions parseOptions(int argc, char *argv[])
{
    StartupOptions rs;
    for (int i = 1; i < argc; i++)
    {
        if (i < argc - 1)
        {
            if (strcmp(argv[i], "-s") == 0)
                rs.epsilon = (float)atof(argv[i + 1]);
            else if (strcmp(argv[i], "-h") == 0)
                rs.tree_size = atoi(argv[i + 1]);
            else if (strcmp(argv[i], "-o") == 0)
                rs.outputFile = argv[i + 1];
            else if (strcmp(argv[i], "-i") == 0)
                rs.inputFile = argv[i + 1];
        }
        if (strcmp(argv[i], "-ps") == 0)
        {
            rs.partitionStyle = PartitionStyle::Static;
        }
        else if (strcmp(argv[i], "-pr") == 0)
        {
            rs.partitionStyle = PartitionStyle::Random;
        }
    }
    return rs;
}

void Image::SaveToFile(std::string& filename) {
  return
}
