#include "fg.h"

Variable::Variable(int x, int y, int color) {
  // Colors: 0-Black | 1-White.
  //TODO: remove redundant variable ?
  position = Vec2<int>(x, y);

  in_msgs[4] = Vec2<float>(color? ENERGY:1.0, color? 1.0:ENERGY);
  belief = Vec2<float>(color? ENERGY:1.0, color? 1.0:ENERGY);
  residual = std::numeric_limits<float>::max();

  // Initialize all messages to 1.
  for (int i = 0; i < 4; i++) {
    in_msgs[i] = {0.5, 0.5};
  }
}

Vec2<float> Variable::calulateBelief() {
  //TODO: whether to normalize
  Vec2<float> belief(1.0, 1.0);
  for (int i = 0; i < 5; i++) {
    belief *= in_msgs[i];
    //printf("v(%d, %d) in_msgs[%d](%f, %f)\n", position.x, position.y, i, in_msgs[i].x, in_msgs[i].y);
  }
  return belief;
}

void Variable::ReceiveMessage(Vec2<float>& msg, int direction) {
  Vec2<float> new_belief = belief / in_msg[j] * msg;
  belief = new_belief;
  in_msg[j] = msg;
  return (new_belief - belief).l1_norm();
}

FactorGraph::FactorGraph(std::vector<std::vector<int>>& img, const char *partitionFile) {
  // img is a 2-d matrix
  width = img[0].size();
  height = img.size();
  variables.resize(width);
  std::ifstream inFile(partitionFile);

  for (int i = 0; i < height; i++) {
    variables.push_back(std::vector<std::shared_ptr<Variable>>());
    for (int j = 0; j < width; j++) {
      std::string line;
      std::getline(inFile, line);
      // std::shared_ptr<Attribute> var = std::make_shared<Variable>(i, j, 
      //                                                             image[i][j]);
      std::shared_ptr<Variable> var = std::make_shared<Variable>(Variable(i, j, img[i][j]));
      //TODO: remove this line
      //var->partition = 2 * (i / 2) + j / 4;
      var->partition = std::stoi(line);
      //var->partition = i * width + j;
      variables[i].push_back(var);
    }
  }
}

std::shared_ptr<Variable> FactorGraph::GetVariable(int i, int j) {
  if (i >= 0 && i < height && j >= 0 && j < width) {
    return variables[i][j];
  }
  return std::shared_ptr<Variable>(nullptr);
}

std::shared_ptr<Variable> FactorGraph::GetNeighbor(std::shared_ptr<Variable> var, int direction) {
  if (direction == 0) {
    ni = v->position.x;
    nj = v->position.y - 1;
  }
  else if (direction == 1) {
    ni = v->position.x + 1;
    nj = v->position.y;
  }
  else if (direction == 2) {
    ni = v->position.x;
    nj = v->position.y + 1;
  }
  else if (direction == 3) {
    ni = v->position.x - 1;
    nj = v->position.y;
  }
  return GetVariable(ni, nj);
}

void FactorGraph::writeDenoisedImage(std::vector<Message>& beliefs, const char* filename) {
    std::ofstream outFile(filename);

    for (Message m : beliefs) {
        Vec2<float> norm_b = m.message.normalize();
            printf("normalized belief for v(%d, %d) is (%f, %f)\n", m.position.x, m.position.y,
            norm_b.x, norm_b.y);
        int color = norm_b.x > norm_b.y ? 0 : 1;

        outFile << m.position.x << " " << m.position.y << " " << color << std::endl;
    }

    outFile.close();
}
