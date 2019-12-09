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


/* 
This function is NOT our work. Copied/Modified from 
https://stackoverflow.com/questions/2654480/writing-bmp-image-in-pure-c-c-without-other-libraries 
See deusmacabre's answer.
*/
void Image::SaveToFile(const char * filename) {

  FILE *f;
  unsigned char *img = NULL;
  int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int

  img = (unsigned char *)malloc(3*w*h);
  memset(img,0,3*w*h);

  for(int i = 0; i < h; i++)
  {
      for(int j = 0; j < w; j++)
      {
          int r = pixels[i][j]*255;
          int g = pixels[i][j]*255;
          int b = pixels[i][j]*255;
          if (r > 255) r = 255;
          if (g > 255) g = 255;
          if (b > 255) b = 255;
          img[(j+i*w)*3+2] = (unsigned char)(r);
          img[(j+i*w)*3+1] = (unsigned char)(g);
          img[(j+i*w)*3+0] = (unsigned char)(b);
      }
  }

  unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
  unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
  unsigned char bmppad[3] = {0,0,0};

  bmpfileheader[ 2] = (unsigned char)(filesize    );
  bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
  bmpfileheader[ 4] = (unsigned char)(filesize>>16);
  bmpfileheader[ 5] = (unsigned char)(filesize>>24);

  bmpinfoheader[ 4] = (unsigned char)(       w    );
  bmpinfoheader[ 5] = (unsigned char)(       w>> 8);
  bmpinfoheader[ 6] = (unsigned char)(       w>>16);
  bmpinfoheader[ 7] = (unsigned char)(       w>>24);
  bmpinfoheader[ 8] = (unsigned char)(       h    );
  bmpinfoheader[ 9] = (unsigned char)(       h>> 8);
  bmpinfoheader[10] = (unsigned char)(       h>>16);
  bmpinfoheader[11] = (unsigned char)(       h>>24);

  f = fopen(filename,"wb");
  fwrite(bmpfileheader,1,14,f);
  fwrite(bmpinfoheader,1,40,f);
  for(int i=0; i<h; i++)
  {
      fwrite(img+(w*(h-i-1)*3),3,w,f);
      fwrite(bmppad,1,(4-(w*3)%4)%4,f);
  }
  free(img);
  fclose(f);
}


StartupOptions parseOptions(int argc, char *argv[])
{
    StartupOptions rs;
    for (int i = 1; i < argc; i++)
    {
        if (i < argc - 1)
        {
            if (strcmp(argv[i], "-e") == 0 || 
                strcmp(argv[i], "--epsilon") == 0)
                rs.converge_threshold = (float)atof(argv[i + 1]);
            else if (strcmp(argv[i], "-h") == 0 || 
                     strcmp(argv[i], "-tree-height") == 0)
                rs.bfs_depth = atoi(argv[i + 1]);
            else if (strcmp(argv[i], "-o") == 0 || 
                     strcmp(argv[i], "--output") == 0)
                rs.output_file = argv[i + 1];
            else if (strcmp(argv[i], "-i") == 0 ||
                     strcmp(argv[i], "--input") == 0)
                rs.input_file = argv[i + 1];
            else if (strcmp(argv[i], "-p") == 0 ||
                     strcmp(argv[i], "--partition-factor") == 0)
                rs.partition_factor = (int) atoi(argv[i + 1]);
        }
    }
    return rs;
}

