#ifndef FG_H_
#define FG_H_

#include <memory>
#include <queue>
#include <vector>
#include <limits>
#include "common.h"

const float ENERGY = 0.1;
const float PHI = 0.3;

struct Message {
  Vec2<float> message;
  Vec2<int> position;
  int direction;
};

class Variable {
public:
  int partition = 0;
  float residual;
  Vec2<float> belief;
  Vec2<int> position;
  Vec2<float> in_msgs[5];
  std::shared_ptr<Variable> neighbors[4];

  Variable(int x, int y, int color); 
  Vec2<float> calulateBelief();

  float ReceiveMessage(Vec2<float>& msg, int direction);

  bool operator < (Variable& var) {
    return residual < var.residual;
  }

  bool operator <= (Variable& var) {
    return residual <= var.residual;
  }

  bool operator == (Variable& var) {
    return residual == var.residual;
  }

  bool operator >= (Variable& var) {
    return residual >= var.residual;
  }

  bool operator > (Variable& var) {
    return residual > var.residual;
  }
};

class FactorGraph {
public:
  int width, height;
  std::vector<std::vector<std::shared_ptr<Variable>>> variables;
  FactorGraph(std::vector<std::vector<int>>& img);
  FactorGraph(std::vector<std::vector<int>>& img, const char* partitionFile);
  std::shared_ptr<Variable> GetVariable(int i, int j);
  std::shared_ptr<Variable> GetNeighbor(std::shared_ptr<Variable> var, int direction);
  static void writeDenoisedImage(std::vector<Message>& beliefs, const char* filename);
};

#endif
