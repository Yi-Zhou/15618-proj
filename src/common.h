#ifndef COMMON_H_
#define COMMON_H_

#include <cstring>
#include <string>
#include <cmath>

const float ENERGY = 0.1;
const float PHI = 0.9;

enum Direction {
  UP = 0,
  RIGHT,
  DOWN,
  LEFT
}

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
    inline float l1_norm(const Vec2<T>& v1, const Vec2<T>& v2) {
      return std::abs(v1.x - v2.x) + std::abs(v1.y - v2.y);
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
  Static, Random;
}

class Image {
public:
  std::vector<int> pixels;
  Image(int w, int h) {
    this->w = w;
    this->h = h;
    this->pixels.resize(w * h);
  }
  SetColor(int i, int j, int color) {
    this->pixels[(i * this->w) + j] = color;
  }
  void SaveToFile(std::string& filename);
};

Image ReadImage(std::string& filename) 
{
  Image img();
  return img;
}

struct StartupOptions
{
    float epsilon = 1e-8;
    int tree_size = 5;
    std::string outputFile = "out.jpeg";
    std::string inputFile = "in.jpeg";
    PartitionStyle partitionStyle = PartitionStyle::Static;
};

StartupOptions parseOptions(int argc, char *argv[]);

