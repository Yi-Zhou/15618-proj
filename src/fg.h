#ifndef FG_H_
#define FG_H_

#include <memory>
#include <queue>
#include <vector>
#include <priority_queue>
#include "common.h"

struct Message {
  int direction;
  Vec2 message;
  std::shared_ptr<Variable> v;
}

class Variable {
public:
  int partition = 0;
  float residual;
  int i, j;
  Vec2<float> belief;
  Vec2<int> position;
  Vec2<float> in_msgs[5];
  int parent;
  std::shared_ptr<Variable> neighbors[4];

  Variable(int x, int y, int color); 
  void SendMessages();

  bool operator< (Variable& var, Variable& var2) {
    return var1.residual < var2.residual;
  }

  bool operator<= (Variable& var, Variable& var2) {
    return var1.residual <= var2.residual;
  }

  bool operator== (Variable& var, Variable& var2) {
    return var1.residual == var2.residual;
  }

  bool operator>= (Variable& var, Variable& var2) {
    return var1.residual >= var2.residual;
  }

  bool operator> (Variable& var, Variable& var2) {
    return var1.residual > var2.residual;
  }
};

class FactorGraph {
public:
  int width, height;
  std::vector<std::vector<std::shared_ptr<Variable>>> varialbles;
  FactorGraph(Image& img);
}

#endif
