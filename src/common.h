#ifndef COMMON_H_
#define COMMON_H_

#include <cstring>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>

const float ENERGY = 0.1;
enum Direction {
  UP = 0,
  RIGHT,
  DOWN,
  LEFT
};

template <class T>
class Vec2
{
public:
    T x, y;
    Vec2() = default;
    Vec2(T vx, T vy)
    {
        x = vx;
        y = vy;
    }
    Vec2<T> normalize() {
        return Vec2(x / (x + y), y / (x + y));
    }
    float l1_norm(const Vec2<T>& vin) {
      return std::abs(x - vin.x) + std::abs(y - vin.y);
    }
    static inline T dot(const Vec2<T> & v0, const Vec2<T> & v1)
    {
        return v0.x * v1.x + v0.y * v1.y;
    }
    inline T & operator [] (int i)
    {
        return ((T*)this)[i];
    }
    inline Vec2<T> operator * (T s) const
    {
        Vec2<T> rs;
        rs.x = x * s;
        rs.y = y * s;
        return rs;
    }
    inline Vec2<T> operator * (const Vec2<T> &vin) const
    {
        Vec2<T> rs;
        rs.x = x * vin.x;
        rs.y = y * vin.y;
        return rs;
    }
    inline Vec2<T> operator / (const Vec2<T> &vin) const
    {
        Vec2<T> rs;
        rs.x = x / vin.x;
        rs.y = y / vin.y;
        return rs;
    }
    inline Vec2<T> operator + (const Vec2<T> &vin) const
    {
        Vec2<T> rs;
        rs.x = x + vin.x;
        rs.y = y + vin.y;
        return rs;
    }
    inline Vec2<T> operator - (const Vec2<T> &vin) const
    {
        Vec2<T> rs;
        rs.x = x - vin.x;
        rs.y = y - vin.y;
        return rs;
    }
    inline Vec2<T> operator -() const
    {
        Vec2<T> rs;
        rs.x = -x;
        rs.y = -y;
        return rs;
    }
    inline Vec2<T> & operator += (const Vec2<T> & vin)
    {
        x += vin.x;
        y += vin.y;
        return *this;
    }
    inline Vec2<T> & operator -= (const Vec2<T> & vin)
    {
        x -= vin.x;
        y -= vin.y;
        return *this;
    }
    Vec2<T> & operator = (T v)
    {
        x = y = v;
        return *this;
    }
    inline Vec2<T> & operator *= (T s)
    {
        x *= s;
        y *= s;
        return *this;
    }
    inline Vec2<T> & operator *= (const Vec2<T> & vin)
    {
        x *= vin.x;
        y *= vin.y;
        return *this;
    }
};

enum class PartitionStyle {
  Static, Random
};

class Image {
public:
  int w, h;
  // std::vector<int> pixels;
  std::vector<std::vector<int>> pixels;

  static Image ReadImage(const char* filename);

  // void SetColor(int i, int j, int color) {
  //   this->pixels[(i * this->w) + j] = color;
  // }
   void SaveToFile(const char* filename);
private:
  Image(std::vector<std::vector<int>>& pixels) {
    this->pixels = pixels;
    this->w = pixels.size();
    this->h = pixels[0].size();
  }
};



struct StartupOptions
{
    float epsilon = 1e-8;
    int tree_size = 5;
    std::string outputFile = "out.jpeg";
    std::string inputFile = "in.jpeg";
    PartitionStyle partitionStyle = PartitionStyle::Static;
};

StartupOptions parseOptions(int argc, char *argv[]);

#endif

