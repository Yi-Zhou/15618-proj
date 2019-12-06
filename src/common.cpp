#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <iomanip>
#include "common.h"

Image Image::ReadImage(const char* imgFile) 
{
  std::ifstream inFile(imgFile);
  std::string delimiter = " ";
  std::string line;
  int i = 0;
  std::vector<std::vector<int>> pixels;
  while (std::getline(inFile, line))
  {
    pixels.push_back(std::vector<int>());
    size_t pos = 0;
    while ((pos = line.find(delimiter)) != std::string::npos) {
      std::string token = line.substr(0, pos);
      int color = std::stoi(token);
      pixels[i].push_back(color);
      //std::cout << token << std::endl;
      line.erase(0, pos + delimiter.length());
    }
    pixels[i].push_back(std::stoi(line));
    i++;
  }  
  return Image(pixels);
}


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
  return;
}
