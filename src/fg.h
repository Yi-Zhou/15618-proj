#ifndef FG_H_
#define FG_H_

#include <memory>
#include <queue>
#include <vector>
#include <limits>
#include "common.h"

struct Message {
  int direction;
  Vec2<float> message;
  Vec2<int> position;
};

class Variable {
public:
  int partition = 0;
  float residual;
  int i, j;
  //Do we really need to store the belief?
  Vec2<float> belief;
  Vec2<int> position;
  Vec2<float> in_msgs[5];
  int parent;
  std::shared_ptr<Variable> neighbors[4];

  Variable(int x, int y, int color); 
  void SendMessages();
  Vec2<float> calulateBelief();

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
  FactorGraph(Image& img);
  FactorGraph(std::vector<std::vector<int>>& img);
};

#endif
