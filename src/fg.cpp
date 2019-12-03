#include "fg.h"

Variable::Variable(int x, int y, int color) {
  // Colors: 0-Black | 1-White.
  //TODO: remove redundant variable ?
  i = x;
  j = y;
  position = Vec2<int>(x, y);

  in_msgs[4] = Vec2<float>(color? ENERGY:1.0, color? 1.0:ENERGY);
  belief = Vec2<float>(color? ENERGY:1.0, color? 1.0:ENERGY);
  residual = std::numeric_limits<float>::max();

  // Initialize all messages to 1.
  for (int i = 0; i < 4; i++) {
    in_msgs[i] = {1.0, 1.0};
  }
}

// void Variable::SendMessages(std::vector<Message> boundary_msgs) {
//   // Update belief.
//   Vec2 new_belief = Vec2<float>(1.0, 1.0)
//   for (int i = 0; i < 4; i++) {
    
//   }
// }
FactorGraph::FactorGraph(std::vector<std::vector<int>>& img) {
  // img is a 2-d matrix
  width = img.size();
  height = img[0].size();
  variables.resize(width);
  for (int i = 0; i < height; i++) {
    variables.push_back(std::vector<std::shared_ptr<Variable>>());
    for (int j = 0; j < width; j++) {
      // std::shared_ptr<Attribute> var = std::make_shared<Variable>(i, j, 
      //                                                             image[i][j]);
      std::shared_ptr<Variable> var = std::make_shared<Variable>(Variable(i, j, img[i][j]));
      //TODO: remove this line
      var->partition = i * height + j;
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

// FactorGraph::FactorGraph(Image& img) {
//   width = img.w;
//   height = img.h;
//   variables.resize(width);
//   for (int i = 0; i < height; i++) {
//     variables.push_back(std::vector<std::shared_ptr<Variable>>());
//     for (int j = 0; j < width; j++) {
//       // std::shared_ptr<Attribute> var = std::make_shared<Variable>(i, j, 
//       //                                                             image[i][j]);
//       std::shared_ptr<Variable> var(i, j, img[i][j]);
//       variables[i].push_back(var);
//     }
//   }

//   for (int i = 0; i < height; i++) {
//     for (int j = 0; j < width; j++) {
//       if (i > 0) {
//         variables[i][j]->neighbors[UP] = variables[i-1][j];
//       }
//       if (j < width - 1) {
//         variables[i][j]->neighbors[RIGHT] = variables[i][j+1];
//       }
//       if (i < height - 1) {
//         variables[i][j]->neighbors[DOWN] = variables[i+1][j];
//       }
//       if (j > 0) {
//         variables[i][j]->neighbors[LEFT] = variables[i][j-1];
//       }
//     }
//   }
// }
