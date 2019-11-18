#include "fg.h"

Variable::Variable(int x, int y, int color) {
  // Colors: 0-Black | 1-White.
  i = x;
  j = y;
  in_msgs[4] = Vec2<float>(color? ENERGY:1.0, color? 1.0:ENERGY);
  belief = Vec2<float>(color? ENERGY:1.0, color? 1.0:ENERGY);
  residual = std::numeric_limits<float>::infinity;

  // Initialize all messages to 1.
  for (int i = 0; i < 4; i++) {
    in_msgs[i] = {1.0, 1.0};
  }
}

void Variable::SendMessages(std::vector<Message> boundary_msgs) {
  // Update belief.
  Vec2 new_belief = Vec2<float>(1.0, 1.0)
  for (int i = 0; i < 4; i++) {
    
  }
}

FactorGraph::FactorGraph(Image& img) {
  width = img.w;
  height = img.h;
  variables.resize(width);
  for (int i = 0; i < height; i++) {
    variables.push_back(std::vector<std::shared_ptr<Variable>>());
    for (int j = 0; j < width; j++) {
      std::shared_ptr<Attribute> var = std::make_shared<Variable>(i, j, 
                                                                  image[i][j]);
      std::shared_ptr<Variable> var(i, j, image[i][j]);
      variables[i].push_back(var);
    }
  }

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      if (i > 0) {
        variables[i][j]->neighbors[UP] = variables[i-1][j];
      }
      if (j < width - 1) {
        variables[i][j]->neighbors[RIGHT] = variables[i][j+1];
      }
      if (i < height - 1) {
        variables[i][j]->neighbors[DOWN] = variables[i+1][j];
      }
      if (j > 0) {
        variables[i][j]->neighbors[LEFT] = variables[i][j-1];
      }
    }
  }
}
